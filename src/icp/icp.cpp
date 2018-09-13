#include "csm/csm_all.h"
#include "icp/icp.h"
#include "egsl/gsl_eigen.h"
#include "egsl/egsl_macros.h"
#include "icp/icp_yogo.h"

#include <math.h>
#include <string.h>
#include <iostream>
#include <chrono>

using namespace std;

void ld_invalid_if_outside(LDP ld, double min_reading, double max_reading) {
    for(int i=0; i<ld->nrays; ++i) {
        if(!ld_valid_ray(ld, i)) continue;
        double r = ld->readings[i];
        if( r <= min_reading || r > max_reading)
            ld->valid[i] = 0;
    }
}

void sm_icp(struct sm_params* params, struct sm_result* result) {
    result->valid = 0;

    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;

//    auto t1 = std::chrono::steady_clock::now();
    if(params->use_corr_tricks || params->debug_verify_tricks)
        ld_create_jump_tables(laser_ref);
//    auto t2 = chrono::steady_clock::now();
//    double dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//    cout << "[icp][test] Cost time of ld_create_jump_tables: " << dt << endl;

    /// @Vance: 转到笛卡尔坐标系下,填充 points 数据, laser_ref 存在重复计算.
//    t1 = std::chrono::steady_clock::now();
    // ld_compute_cartesian(laser_ref);
    ld_compute_cartesian(laser_sens);
//    t2 = chrono::steady_clock::now();
//    dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//    cout << "[icp][test] Cost time of ld_compute_cartesian: " << dt << endl;

    // 简单分割，ld->cluster[i]存储了第i个点对应的点云簇序号
    if(params->do_alpha_test) {
        ld_simple_clustering(laser_ref, params->clustering_threshold);
        ld_compute_orientation(laser_ref, params->orientation_neighbourhood, params->sigma);
        ld_simple_clustering(laser_sens, params->clustering_threshold);
        ld_compute_orientation(laser_sens, params->orientation_neighbourhood, params->sigma);
    }

    gsl_vector * x_new = gsl_vector_alloc(3);
    gsl_vector * x_old = vector_from_array(3, params->first_guess);

    if(params->do_visibility_test) {
        sm_debug("laser_ref:\n");
        visibilityTest(laser_ref, x_old);

        sm_debug("laser_sens:\n");
        gsl_vector * minus_x_old = gsl_vector_alloc(3);
        ominus(x_old, minus_x_old);
        visibilityTest(laser_sens, minus_x_old);
        gsl_vector_free(minus_x_old);
    }

    double error;
    int nvalid;
    int iterations;
//    t1 = std::chrono::steady_clock::now();
    if(!icp_loop(params, x_old->data(), x_new->data(), &error, &nvalid, &iterations)) {
        sm_error("icp: ICP failed for some reason. \n");
        result->valid = 0;
        result->iterations = iterations;
        result->nvalid = nvalid;
    } else {
        /* It was succesfull */
        int restarted = 0;
        double best_error = error;
        gsl_vector * best_x = gsl_vector_alloc(3);
        gsl_vector_memcpy(best_x, x_new);

        if(error/nvalid > params->restart_threshold_mean_error ) {
            std::cerr << "Too large mean dist: " << error/nvalid << ", icp failed." << std::endl;

            result->valid = 0;
            result->iterations = iterations;
            result->nvalid = nvalid;
            gsl_vector_free(x_new);
            gsl_vector_free(x_old);

            return;
        }

        if(params->restart &&
           error/nvalid > params->restart_threshold_mean_error ) {
            sm_debug("Restarting: %f > %f \n",(error/nvalid),(params->restart_threshold_mean_error));
            restarted = 1;
            double dt  = params->restart_dt;
            double dth = deg2rad(params->restart_dtheta);
            sm_debug("icp_loop: dt = %f dtheta= %f deg\n", dt, rad2deg(dth));

            double perturb[6][3] = {
                {dt,0,0}, {-dt,0,0},
                {0,dt,0}, {0,-dt,0},
                {0,0,dth}, {0,0,-dth}
            };

            /// @Vance: 从六个方向上±x，±y，±f=α更新估计的相对位姿再进行icp匹配
            int a; for(a=0; a<6; a++){
                sm_debug("-- Restarting with perturbation #%d\n", a);
                struct sm_params my_params = *params;
                gsl_vector * start = gsl_vector_alloc(3);
                    gvs(start, 0, gvg(x_new,0)+perturb[a][0]);
                    gvs(start, 1, gvg(x_new,1)+perturb[a][1]);
                    gvs(start, 2, gvg(x_new,2)+perturb[a][2]);
                gsl_vector * x_a = gsl_vector_alloc(3);
                double my_error; int my_valid; int my_iterations;
                if(!icp_loop(&my_params, start->data(), x_a->data(), &my_error, &my_valid, &my_iterations)){
                    sm_error("Error during restart #%d/%d. \n", a, 6);
                    break;
                }
                iterations += my_iterations;
                if (iterations > params->max_iterations) break;

                if(my_error < best_error) {
                    sm_debug("--Perturbation #%d resulted in error %f < %f\n", a,my_error,best_error);
                    gsl_vector_memcpy(best_x, x_a);
                    best_error = my_error;
                }
                gsl_vector_free(x_a);
                gsl_vector_free(start);
            }
        }


        /* At last, we did it. */
        result->valid = 1;
        vector_to_array(best_x, result->x);
        sm_debug("icp: final x =  %s  \n", gsl_friendly_pose(best_x));

        if (restarted) { // recompute correspondences in case of restarts
            ld_compute_world_coords(laser_sens, result->x);
            if(params->use_corr_tricks)
                find_correspondences_tricks(params);
            else
                find_correspondences(params);
        }

        if(params->do_compute_covariance)  {
            val cov0_x, dx_dy1, dx_dy2;
            compute_covariance_exact(
                laser_ref, laser_sens, best_x,
                &cov0_x, &dx_dy1, &dx_dy2);

            val cov_x = sc(square(params->sigma), cov0_x); // scale * matrix
    /*			egsl_v2da(cov_x, res->cov_x); */

            result->cov_x_m = egsl_v2gslm(cov_x);
            result->dx_dy1_m = egsl_v2gslm(dx_dy1);
            result->dx_dy2_m = egsl_v2gslm(dx_dy2);
        }

        result->error = best_error;
        result->iterations = iterations;
        result->nvalid = nvalid;

        gsl_vector_free(best_x);
    }
//    t2 = chrono::steady_clock::now();
//    dt = chrono::duration<double, ratio<1,1000000>>(t2 - t1).count();
//    cout << "[icp][test] Cost time of icp_loop: " << dt << endl;

    /// @Vance: 对每次算出的变换结果做个有效性检查
    if (!valid_transform(params, result->x))
        result->valid = 0;

    gsl_vector_free(x_new);
    gsl_vector_free(x_old);
}

void csm_free_unused_memory(){
    egsl_free_unused_memory();
}
