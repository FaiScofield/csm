#include "csm/csm_all.h"
#include "icp/icp_optimization.h"
#include "icp/icp_yogo.h"
#include <iostream>
#include <fstream>
#include <thread>


using namespace std;

/** for the need of new keyframe judgment */
bool new_keyframe_needed(const sm_params* param, const double* trans)
{
    if (param->kf_delta_frame < 2)
        return true;
    if (param->kf_dist_linear == 0 || param->kf_dist_angular == 0)
        return true;
    if (fabs(trans[2]) > param->kf_dist_angular)
        return true;

    double x = trans[0];
    double y = trans[1];
    double kf_dist_linear_sq = param->kf_dist_linear * param->kf_dist_linear;
    if (x*x + y*y > kf_dist_linear_sq)
        return true;

    return false;
}

/** whether need optimization or not */
bool need_optimization(const sm_params* params, const vector<laser_data>& keyframes,
                      const unsigned long bi, const unsigned long li)
{
    if (li - bi < 3) return false;
    if (li - bi + 1 >= params->pg_max_frames) {
        cout << "[icp][optimization] need optimization for frames reach to " << li-bi+1
             << ". optimize from #" << bi << "# to #" << li << endl;
        return true;
    }

    double dx = abs(keyframes[li].true_pose[0] - keyframes[bi].true_pose[0]);
    double dy = abs(keyframes[li].true_pose[1] - keyframes[bi].true_pose[1]);
    double d = dx * dx + dy * dy;
    if (d < 1) return false;
    else if (d > 3) {
        cout << "[icp][optimization] need optimization for d = " << d
             << " > 3m, get " << li-bi+1 << " frames to optimize."
             << ". from #" << bi << "# to #" << li << endl;
        return true;
    }

    double da = angleDiff(keyframes[li].true_pose[2], keyframes[bi].true_pose[2]);
    if (abs(da) > deg2rad(30)) {
        cout << "[icp][optimization] need optimization for da = " << da
             << " > 30 deg, index from " << bi << " to " << li
             << ", get " << li-bi+1 << " frames to optimize."
             << ". from #" << bi << "# to #" << li << endl;
        return true;
    }

    return false;
}

/** compute weight for edge's information matrix */
double compute_weight_new_edge(const double* gp_i, const double* gp_j,
                               const double* trans, const int iterations)
{
    double w = 1.0;
    if (iterations <= 5 ) return w;
    else {
        w = std::max( 1.0-(iterations-5)/50.0, 0.0 );
        if (iterations <= 10) return w;

//        double da =gp_j[2] - gp_i[2];
        double da = angleDiff(gp_j[2], gp_i[2]);
        double dx = gp_j[0] - gp_i[0] * cos(da) + gp_i[1] * sin(da);
        double dy = gp_j[1] - gp_i[0] * sin(da) - gp_i[1] * cos(da);
        double c = (dx - trans[0]) * (dx - trans[0]) +
                   (dy - trans[1]) * (dy - trans[1]) +
                   (da - trans[2]) * (da - trans[2]);
        c = 0.1 / (0.1 + c);
        w = 0.7 * w + 0.3 * c;

        return w;
    }
}

/** compute weight for edge's information matrix */
double compute_weight_neighbor_edge(const int iterations)
{
    double w = 1.0;
    if (iterations > 5)
        w = std::max( 1.0-(iterations-5)/50.0, 0.0 );

    return w;
}

/** find extra edges to optimize the icp results */
int add_extra_edges(sm_params* params, vector<icp_edge>& edges,
                    const vector<laser_data>& keyframes,
                    const unsigned long bi, const unsigned long li)
{
    sm_result result;

    int nDisFilter, nAngFilter, nIterFilter, newEdges, tatal_count, npgframes;
    nDisFilter = nAngFilter =  nIterFilter = newEdges = tatal_count = 0;
    npgframes = li - bi + 1;

    /**
     * delta_angule < 20 degree to include
     * delta_distance < 0.12*kf_delta_frame meters to include
     * iterations < 25 times to include
     **/
    int nrays = keyframes[0].nrays;
    LDP kf1 = ld_alloc_new(nrays);
    LDP kf2 = ld_alloc_new(nrays);
    for (int i=0; i<npgframes-2; ++i) {
        int j = i + 2;
        ld_from_keyframe(kf1, keyframes[bi+i]);
        ld_from_keyframe(kf2, keyframes[bi+j]);
        tatal_count++;

        double da = angleDiff(kf2->true_pose[2], kf1->true_pose[2]);
        if (abs(da) > deg2rad(20)) {
            ++nAngFilter;
            continue;
        }

        double dx = abs(kf2->true_pose[0] - kf1->true_pose[0]);
        double dy = abs(kf2->true_pose[1] - kf1->true_pose[1]);
        double d = dx * dx + dy * dy;
        double d_max = params->kf_delta_frame * params->max_linear_velocity * params->laser_frequency;
        if (d > d_max) {
            ++nDisFilter;
            continue;
        }

        params->laser_ref = kf1;
        params->laser_sens = kf2;
        set_first_guess(params);
//        double odometry[3], ominus_laser[3], temp[3];
//        pose_diff_d(kf2->odometry, kf1->odometry, odometry);
//        copy_d(keyframes[bi+i+1].last_trans, 2, odometry);
//        ominus_d(params->laser, ominus_laser);
//        oplus_d(ominus_laser, odometry, temp);
//        oplus_d(temp, params->laser, params->first_guess);

//        std::cout << "[icp][optimization] Computong extra edges from #"
//                  << bi+i << "# to #" << bi+j << "#" << std::endl;
        sm_icp(params, &result);
        if (result.iterations > params->max_iterations/2 || !result.valid) {
//            printf("[icp][optimization] drop edges: #%lu and #%lu\n", bi+i, bi+j);
            ++nIterFilter;
            continue;
        }
//        if ( !valid_transform(params, result.x) ){
//            printf("[icp][optimization] drop edges: #%lu and #%lu\n", bi+i, bi+j);
//            ++nDisFilter;
//            continue;
//        }

        /// finally a new edge born
        ++newEdges;
        icp_edge edge;
        edge.edge_id_from = i;
        edge.edge_id_to   = j;
        copy_d(result.x, 3, edge.transform);
        double w = compute_weight_new_edge(kf1->true_pose, kf2->true_pose,
                                           result.x, result.iterations);

        // use fixed weight
        edge.information[0] = 20 * w;
        edge.information[1] = 5 * w;
        edge.information[2] = 286.4789 * w; // 900/pi, 286.4789
        edges.push_back(edge);
    }
    ld_free(kf1);
    ld_free(kf2);

    printf("[icp][optimization] Add tatal new edges: %d / %d\n", newEdges, tatal_count);
//    printf("[icp] discard edge matching through distance: %d)\n", nDisFilter);
//    printf("[icp] discard edge matching through angle: %d\n", nAngFilter);
//    printf("[icp] discard edge matching through iterations: %d\n", nIterFilter);
    if (tatal_count != (li-bi-1))
        cerr << "[icp][optimization] ***** error: tatal_count != (li-bi-1) \n";

    return newEdges;
}


/** write the vertexs and edges to a g2o file */
void write_to_g2o_file(const vector<icp_vertex>& vertexs, const vector<icp_edge>& edges,
                       const string g2o_file)
{
    ofstream fout;
    try {
        fout.open(g2o_file.c_str());
        fout.close();
        fout.open(g2o_file.c_str(), ios::app);
        for (auto& v : vertexs) {
            fout << "VERTEX_SE2 " << v.vertex_id << " " << v.vertex_pose[0] << " "
                 << v.vertex_pose[1] << " " << v.vertex_pose[2] << "\n";
        }
        for (auto& e : edges) {
            fout << "EDGE_SE2 " << e.edge_id_from << " " << e.edge_id_to << " "
                 << e.transform[0] << " " << e.transform[1] << " " << e.transform[2] << " "
                 << e.information[0] << " 0 0 " << e.information[1] << " 0 "
                 << e.information[2] << "\n";
        }
        fout.close();
    } catch(...) {
        printf("[icp][error] write to g2o file failed!\n");
    }
    printf("[icp][info] write to g2o file sucessed: %s\n", g2o_file.c_str());
}
