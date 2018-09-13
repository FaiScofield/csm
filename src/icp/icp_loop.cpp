#include "egsl/gsl_eigen.h"
#include "egsl/egsl_macros.h"
#include "gpc/gpc.h"
#include "csm/csm_all.h"
#include "icp/icp.h"

#include <iostream>
#include <math.h>
#include <string.h>
#include <vector>
#include <chrono>
//#include <thread>
#include <fstream>

using namespace std;

std::string pre = "./0824-8/";

int icp_loop(struct sm_params* params, const double* q0, double* x_new,
    double* total_error, int* nvalid, int* iterations)
{
    std::ofstream file_out;

    if(any_nan(q0, 3)) {
        std::cerr << "[icp][error] Initial guess contains nan! \n";
        return 0;
    }

    LDP laser_ref = params->laser_ref;  /// @Vance
    LDP laser_sens = params->laser_sens;
    double x_old[3], delta[3], delta_old[3] = {0, 0, 0};
    copy_d(q0, 3, x_old);

    /// @Vance: 保存每次迭代的中间量做可视化
//    cout << "[icp][loop] Icp loop from #" << laser_sens->frame_id << "# to #" << laser_ref->frame_id
//         << "# with first guess: " << x_old[0] << ", " << x_old[1] << ", " << x_old[2] << endl;
    if (params->icp_iteration_visible) {
        std::stringstream ss;
        ss << pre << "frame-" << laser_ref->frame_id << "-ref.txt";
        try {
            file_out.open(ss.str());
            file_out.close();
            file_out.open(ss.str(), std::ios::app);
            for (int i=0; i<laser_ref->nrays; ++i) {
                file_out << laser_ref->points[i].p[0] << " "
                     << laser_ref->points[i].p[1]  << "\n";
            }
            file_out.close();
        } catch(...) {
            printf("[icp][error] write to icp iteration refference file failed!\n");
        }
        std::stringstream s;
        s << pre << "frame-" << laser_sens->frame_id << "-sen.txt";
        try {
            file_out.open(s.str());
            file_out.close();
            file_out.open(s.str(), std::ios::app);
            for (int i=0; i<laser_sens->nrays; ++i) {
                file_out << laser_sens->points[i].p[0] << " "
                     << laser_sens->points[i].p[1]  << "\n";
            }
            file_out.close();
        } catch(...) {
            printf("[icp][error] write to icp iteration refference file failed!\n");
        }
    }

    //unsigned int hashes[params->max_iterations];
    std::vector<unsigned int> hashes(params->max_iterations, 0);
    int iteration;

    sm_debug("icp: starting at  q0 =  %s  \n", friendly_pose(x_old));

    int all_is_okay = 1;


    for(iteration=0; iteration<params->max_iterations; ++iteration) {
        sm_debug("== icp_loop: starting iteration. %d  \n", iteration);

        /** Compute laser_sens's points in laser_ref's coordinates
            by roto-translating by x_old */
        /// @Vance: 实际上是根据相对变换转到相对帧坐标系下,填充 points_w 数据
//        auto t1 = std::chrono::steady_clock::now();
        ld_compute_world_coords(laser_sens, x_old);
//        auto t2 = chrono::steady_clock::now();
//        double dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//        cout << "[icp][test][loop] Cost time of ld_compute_world_coords: " << dt << endl;

        /** Find correspondences (the naif or smart way) */
//        t1 = std::chrono::steady_clock::now();
        if(params->use_corr_tricks)
            find_correspondences_tricks(params);
        else
            find_correspondences(params);
//        t2 = chrono::steady_clock::now();
//        dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//        cout << "[icp][test][loop] Cost time of find_correspondences_tricks: " << dt << endl;


        /// @Vance: 保存每次迭代的中间量做可视化
        if (params->icp_iteration_visible) {
            std::stringstream ss;
            ss << pre << "frame-" << laser_sens->frame_id << "-to-"
               << laser_ref->frame_id << "-iter-" << iteration << ".txt";
            try {
                file_out.open(ss.str());
                file_out.close();
                file_out.open(ss.str(), std::ios::app);
                for (int i=0; i<laser_sens->nrays; ++i) {
                    file_out << laser_sens->points_w[i].p[0] << " "
                         << laser_sens->points_w[i].p[1] << " "
                         << laser_sens->corr->j1 << " "
                         << laser_sens->corr->j2 << "\n";
                }
                file_out.close();
            } catch(...) {
                printf("[icp][error] write to icp iteration file failed!\n");
            }
        }

        /** If debug_verify_tricks, make sure that find_correspondences_tricks()
            and find_correspondences() return the same answer */
        if(params->debug_verify_tricks)
            debug_correspondences(params);

        /** If not many correspondences, bail out */
        int num_corr = ld_num_valid_correspondences(laser_sens);
        double fail_perc = params->corr_fail_perc;
        if(num_corr < fail_perc * laser_sens->nrays) {
            std::cout << "[icp][error] Current frame " << laser_sens->frame_id
                      << " error for less corr: " << num_corr << std::endl;
            sm_error("	: too less correspondences, only %d.\n", num_corr);
            all_is_okay = 0;
            break;
        }

        /** Kill some correspondences (using dubious algorithm) */
//        t1 = std::chrono::steady_clock::now();
        if(params->outliers_remove_doubles)
            kill_outliers_double(params);
//        t2 = chrono::steady_clock::now();
//        dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//        cout << "[icp][test][loop] Cost time of kill_outliers_double: " << dt << endl;

        double error = 0;
        /** Trim correspondences */
//        t1 = std::chrono::steady_clock::now();
        kill_outliers_trim(params, &error);
        int num_corr_after = ld_num_valid_correspondences(laser_sens);
//        t2 = chrono::steady_clock::now();
//        dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//        cout << "[icp][test][loop] Cost time of kill_outliers_trim: " << dt << endl;

        *total_error = error;
        *nvalid = num_corr_after;

        /** If not many correspondences, bail out */
        if (num_corr_after < fail_perc * laser_sens->nrays){
            sm_error("icp_loop: failed: after trimming, only %d correspondences.\n", num_corr_after);
            all_is_okay = 0;
            break;
        }

        /** Compute next estimate based on the correspondences */
//        t1 = std::chrono::steady_clock::now();
        if(!compute_next_estimate(params, x_old, x_new)) {
            sm_error("  icp_loop: Cannot compute next estimate.\n");
            all_is_okay = 0;
            break;
        }
//        t2 = chrono::steady_clock::now();
//        dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//        cout << "[icp][test][loop] Cost time of compute_next_estimate: " << dt << endl;
//        cout << "[icp][test][loop] x_new from orig: " << x_new[0] << ", " << x_new[1] << ", " << x_new[2] << endl;

        // cai
//        t1 = std::chrono::steady_clock::now();
//        compute_next_estimate_cai(params, x_new);
//        t2 = chrono::steady_clock::now();
//        dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//        cout << "[icp][test][loop] Cost time of compute_next_estimate_SVD: " << dt << endl;
//        cout << "[icp][test][loop] x_new from SVD: " << x_new[0] << ", " << x_new[1] << ", " << x_new[2] << endl;

        pose_diff_d(x_new, x_old, delta);

        /** Checks for oscillations */
        hashes[iteration] = ld_corr_hash(laser_sens);

        /** PLICP terminates in a finite number of steps! */
        if(params->use_point_to_line_distance) {
            int loop_detected = 0; /* TODO: make function */
            int a;
            for(a=iteration-1; a>=0; a--) {
                if(hashes[a] == hashes[iteration]) {
                    sm_debug("icpc: oscillation detected (cycle length = %d)\n", iteration-a);
                    loop_detected = 1;
                    break;
                }
            }
            if(loop_detected) {
                break;
            }
        }

        /** This termination criterium is useless when using
           the point-to-line-distance; however, we put it here because
           one can choose to use the point-to-point distance. */
        if(termination_criterion(params, delta)) {
            break;
        }

        copy_d(x_new, 3, x_old);
        copy_d(delta, 3, delta_old);
    }

    /// @Vance: 迭代次数从0开始算，这里补上1
    *iterations = iteration + 1;

    return all_is_okay;
}

/// @Vance: 终止标准
int termination_criterion(struct sm_params*params, const double*delta){
    double a = norm_d(delta);
    double b = fabs(delta[2]);
    return (a < params->epsilon_xy) && (b < deg2rad(params->epsilon_theta));
}

int compute_next_estimate(struct sm_params* params, const double x_old[3], double x_new[3])
{
    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;

    //struct gpc_corr c[laser_sens->nrays];
    struct gpc_corr dummy;
    std::vector<gpc_corr> c(laser_sens->nrays, dummy);

    int i; int k=0;
    for(i=0; i<laser_sens->nrays; ++i) {
        if(!laser_sens->valid[i])
            continue;

        if(!ld_valid_corr(laser_sens, i))
            continue;

        int j1 = laser_sens->corr[i].j1;
        int j2 = laser_sens->corr[i].j2;

        c[k].valid = 1;

        if(laser_sens->corr[i].type == correspondence::corr_pl) {
            c[k].p[0] = laser_sens->points[i].p[0];
            c[k].p[1] = laser_sens->points[i].p[1];
            c[k].q[0] = laser_ref->points[j1].p[0];
            c[k].q[1] = laser_ref->points[j1].p[1];

            /** TODO: here we could use the estimated alpha */
            double diff[2];
            diff[0] = laser_ref->points[j1].p[0] - laser_ref->points[j2].p[0];
            diff[1] = laser_ref->points[j1].p[1] - laser_ref->points[j2].p[1];
            double one_on_norm = 1 / sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
            double normal[2];
            normal[0] = +diff[1] * one_on_norm;
            normal[1] = -diff[0] * one_on_norm;

            double cos_alpha = normal[0];
            double sin_alpha = normal[1];

            c[k].C[0][0] = cos_alpha * cos_alpha;
            c[k].C[1][0] =
            c[k].C[0][1] = cos_alpha * sin_alpha;
            c[k].C[1][1] = sin_alpha * sin_alpha;

#if 1
            /* Note: it seems that because of numerical errors this matrix might be
               not semidef positive. 半正定 */
            double det = c[k].C[0][0] * c[k].C[1][1] - c[k].C[0][1] * c[k].C[1][0];
            double trace = c[k].C[0][0] + c[k].C[1][1];

            bool semidef = (det >= 0) && (trace > 0);
            if(!semidef) {
//                printf("%d: Adjusting correspondence weights\n",i);
                double eps = -det;
                c[k].C[0][0] += 2 * sqrt(eps);
                c[k].C[1][1] += 2 * sqrt(eps);
            }
#endif
        } else {
            c[k].p[0] = laser_sens->points[i].p[0];
            c[k].p[1] = laser_sens->points[i].p[1];

            projection_on_segment_d(
                laser_ref->points[j1].p,
                laser_ref->points[j2].p,
                laser_sens->points_w[i].p,
                c[k].q);

            /* Identity matrix */
            c[k].C[0][0] = 1;
            c[k].C[1][0] = 0;
            c[k].C[0][1] = 0;
            c[k].C[1][1] = 1;
        }


        double factor = 1;

        /* Scale the correspondence weight by a factor concerning the
           information in this reading. */
        if(params->use_ml_weights) {
            int have_alpha = 0;
            double alpha = 0;
            if(!is_nan(laser_ref->true_alpha[j1])) {
                alpha = laser_ref->true_alpha[j1];
                have_alpha = 1;
            } else if(laser_ref->alpha_valid[j1]) {
                alpha = laser_ref->alpha[j1];;
                have_alpha = 1;
            } else have_alpha = 0;

            if(have_alpha) {
                double pose_theta = x_old[2];
                /** Incidence of the ray
                    Note that alpha is relative to the first scan (not the world)
                    and that pose_theta is the angle of the second scan with
                    respect to the first, hence it's ok. */
                double beta = alpha - (pose_theta + laser_sens->theta[i]);
                factor = 1 / square(cos(beta));
            } else {
                static int warned_before = 0;
                if(!warned_before) {
                    sm_error("Param use_ml_weights was active, but not valid alpha[] or true_alpha[]."
                              "Perhaps, if this is a single ray not having alpha, you should mark it as inactive.\n");
                    sm_error("Writing laser_ref: \n");
                    /*ld_write_as_json(laser_ref, stderr);*/
                    warned_before = 1;
                }
            }
        }

        /* Weight the points by the sigma in laser_sens */
        if(params->use_sigma_weights) {
            if(!is_nan(laser_sens->readings_sigma[i])) {
                factor *= 1 / square(laser_sens->readings_sigma[i]);
            } else {
                static int warned_before = 0;
                if(!warned_before) {
                    sm_error("Param use_sigma_weights was active, but the field readings_sigma[] was not filled in.\n");
                    sm_error("Writing laser_sens: \n");
                    /*ld_write_as_json(laser_sens, stderr);*/
                }
            }
        }

        c[k].C[0][0] *= factor;
        c[k].C[1][0] *= factor;
        c[k].C[0][1] *= factor;
        c[k].C[1][1] *= factor;

        k++;
    }

    /* TODO: use prior for odometry */
    double std = 0.11;
    const double inv_cov_x0[9] =
        {1/(std*std), 0, 0,
         0, 1/(std*std), 0,
         0, 0, 0};

    /// @Vance: 根据最小二乘计算新的估计，这里很重要，但是看不懂...
    int ok = gpc_solve(k, c, 0, inv_cov_x0, x_new);
    if(!ok) {
        sm_error("gpc_solve_valid failed\n");
        return 0;
    }

    double old_error = gpc_total_error(c, k, x_old);
    double new_error = gpc_total_error(c, k, x_new);

    sm_debug("\tcompute_next_estimate: old error: %f  x_old= %s \n", old_error, friendly_pose(x_old));
    sm_debug("\tcompute_next_estimate: new error: %f  x_new= %s \n", new_error, friendly_pose(x_new));
    sm_debug("\tcompute_next_estimate: new error - old_error: %g \n", new_error-old_error);

    double epsilon = 0.000001;
    if(new_error > old_error + epsilon) {
        std::cout << "[icp][loop] Error when computing next estimate between #"
                  << laser_ref->frame_id << "# and #" << laser_sens->frame_id <<  std::endl;
        sm_error("\tcompute_next_estimate: something's fishy here! Old error: %lf  new error: %lf  x_old %lf %lf %lf x_new %lf %lf %lf\n",old_error,new_error,x_old[0],x_old[1],x_old[2],x_new[0],x_new[1],x_new[2]);
//        return 0;
    }

    return 1;
}
	

/// @Vance: test
int compute_next_estimate_SVD(struct sm_params* params, double x_new[3])
{
    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;
    int nrays = laser_sens->nrays;
    int N = 0;
    Eigen::Vector2d center_mass_ref = Eigen::Vector2d(0., 0.);
    Eigen::Vector2d center_mass_sens= Eigen::Vector2d(0., 0.);

    for (int i=0; i<nrays; ++i) {
        if ( !ld_valid_corr(laser_sens, i) ) continue;

        center_mass_sens[0] += laser_sens->points_w[i].p[0];
        center_mass_sens[1] += laser_sens->points_w[i].p[1];
        int j1 = laser_sens->corr[i].j1;
        center_mass_ref[0] += laser_ref->points[j1].p[0];
        center_mass_ref[1] += laser_ref->points[j1].p[1];

        N++;
    }

    center_mass_ref /= N;
    center_mass_sens /= N;

    vector<Eigen::Vector2d> points_ref, points_sens;
    for (int i=0; i<nrays; ++i) {
        if ( !ld_valid_corr(laser_sens, i) ) continue;
        int j1 = laser_sens->corr[i].j1;
        Eigen::Vector2d point_sens = Eigen::Vector2d(laser_sens->points_w[i].p[0] - center_mass_sens[0],
                                laser_sens->points_w[i].p[1] - center_mass_sens[1]);
        Eigen::Vector2d point_ref = Eigen::Vector2d(laser_ref->points[j1].p[0] - center_mass_ref[0],
                               laser_ref->points[j1].p[1] - center_mass_ref[1]);
        points_sens.push_back(point_sens);
        points_ref.push_back(point_ref);
    }

    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    for (int i=0; i<N; ++i)
        W += points_sens[i] * points_ref[i].transpose();

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixX2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = U * V.transpose();
    Eigen::Vector2d t = center_mass_sens - R * center_mass_ref;

    x_new[0] = t(0,0);
    x_new[1] = t(1,0);
    x_new[2] = acos(R(0,0));
}

/// @Vance: test
int compute_next_estimate_cai(struct sm_params* params, double x_new[3])
{
    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;
    int nrays = laser_sens->nrays;
    int N = 0;
    Eigen::Vector2d center_mass_ref = Eigen::Vector2d(0., 0.);
    Eigen::Vector2d center_mass_sens= Eigen::Vector2d(0., 0.);
    Eigen::Vector2d tatal_mass_ref = Eigen::Vector2d(0., 0.);
    Eigen::Vector2d tatal_mass_sens= Eigen::Vector2d(0., 0.);

    double SxAxB, SyAyB, SxAyB, SyAxB;
    SxAxB = SyAyB = SxAyB = SyAxB = 0;
    for (int i=0; i<nrays; ++i) {
        if ( !ld_valid_corr(laser_sens, i) ) continue;

        tatal_mass_sens[0] += laser_sens->points_w[i].p[0];
        tatal_mass_sens[1] += laser_sens->points_w[i].p[1];
        int j1 = laser_sens->corr[i].j1;
        tatal_mass_ref[0] += laser_ref->points[j1].p[0];
        tatal_mass_ref[1] += laser_ref->points[j1].p[1];

        SxAxB += laser_sens->points_w[i].p[0] * laser_ref->points[j1].p[0];
        SyAyB += laser_sens->points_w[i].p[1] * laser_ref->points[j1].p[1];
        SxAyB += laser_sens->points_w[i].p[0] * laser_ref->points[j1].p[1];
        SyAxB += laser_sens->points_w[i].p[1] * laser_ref->points[j1].p[0];

        N++;
    }

    center_mass_ref = tatal_mass_ref / N;
    center_mass_sens = tatal_mass_sens / N;
    SxAxB -= tatal_mass_ref[0] * center_mass_sens[0];
    SyAyB -= tatal_mass_ref[1] * center_mass_sens[1];
    SxAyB -= tatal_mass_ref[0] * center_mass_sens[0];
    SyAxB -= tatal_mass_ref[1] * center_mass_sens[1];

    double dth = atan2((SxAyB - SyAxB), (SxAxB + SyAyB));
    x_new[2] = dth;
    x_new[0] = center_mass_sens[0] - center_mass_ref[0]*cos(dth) + center_mass_ref[1]*sin(dth);
    x_new[1] = center_mass_sens[1] - center_mass_ref[0]*sin(dth) - center_mass_ref[1]*cos(dth);

}
