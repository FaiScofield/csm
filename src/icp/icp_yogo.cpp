#include "icp/icp_yogo.h"
#include "csm/csm_all.h"
#include "csm/math_utils.h"

#include <iostream>

using namespace std;


/** set params */
void set_plicp_params(sm_params* plicp_param)
{
    plicp_param->max_angular_correction_deg = 45.0;
    plicp_param->max_linear_correction = 0.5;
    plicp_param->max_iterations = 50;    // 1000
    plicp_param->epsilon_xy = 0.0001;    // A threshold for stopping icp loop
    plicp_param->epsilon_theta = 0.005; // A threshold for stopping icp loop
    plicp_param->max_correspondence_dist = 0.25;
    plicp_param->sigma = 0.005;
    plicp_param->use_corr_tricks = 1;
    plicp_param->restart = 1;
    plicp_param->restart_threshold_mean_error = 0.01;
    plicp_param->restart_dt = 0.01;
    plicp_param->restart_dtheta = deg2rad(1.5);
    plicp_param->clustering_threshold = 0.05;
    plicp_param->orientation_neighbourhood = 10;
    plicp_param->use_point_to_line_distance = 1;
    plicp_param->do_alpha_test = 0;
    plicp_param->do_alpha_test_thresholdDeg = 20.0;
    plicp_param->outliers_maxPerc = 0.95;
    plicp_param->outliers_adaptive_mult = 2.0;
    plicp_param->outliers_adaptive_order = 0.7;
    plicp_param->do_visibility_test = 0;
    plicp_param->outliers_remove_doubles = 1;
    plicp_param->do_compute_covariance = 1;
    plicp_param->debug_verify_tricks = 0;
    plicp_param->min_reading = 0.0;
    plicp_param->max_reading = 10.0;
    plicp_param->use_ml_weights = 0;
    plicp_param->use_sigma_weights = 0;
    plicp_param->laser[0] = 0.117;
    plicp_param->laser[1] = 0.0;
    plicp_param->laser[2] = 0.0;
    plicp_param->kf_delta_frame = 1;
    plicp_param->kf_dist_linear = 0.025;
    plicp_param->kf_dist_angular = 0.05;
    plicp_param->pg_max_iterations = 5;
    plicp_param->pg_max_frames = 100;

    plicp_param->min_valid_distance = 0.0015;
    plicp_param->min_valid_angle = 0.2;
    plicp_param->max_linear_velocity = 2.0;
    plicp_param->max_angular_velocity = 3.0;
    plicp_param->laser_frequency = 0.06;
    plicp_param->corr_fail_perc  = 0.05;
    plicp_param->icp_iteration_visible = 0;
}

/** get global position */
void get_global_pose(double* global_pos, const double* gp_ref, const double* delta_trans)
{
    double dx = delta_trans[0];
    double dy = delta_trans[1];
    double da = delta_trans[2];
    double gp_curr[3];
    gp_curr[0] = cos(gp_ref[2])*dx - sin(gp_ref[2])*dy + gp_ref[0];
    gp_curr[1] = sin(gp_ref[2])*dx + cos(gp_ref[2])*dy + gp_ref[1];
    gp_curr[2] = gp_ref[2] + da;
    if (gp_curr[2] > M_PI) gp_curr[2] -= 2 * M_PI;
    if (gp_curr[2] < -M_PI) gp_curr[2] += 2 * M_PI;

    copy_d(gp_curr, 3, global_pos);
}

bool valid_transform(sm_params* const params, double* delta_trans)
{
    double dx = delta_trans[0];
    double dy = delta_trans[1];
    double da = abs(delta_trans[2]);
    double dsq = dx * dx + dy * dy;

    // deal with too large motion
    double max_dis = params->kf_delta_frame * params->max_linear_velocity * params->laser_frequency;
    double max_angle = params->kf_delta_frame * params->max_angular_velocity * params->laser_frequency;
    if (dsq > max_dis*max_dis || da > max_angle) {
        cout << "[icp][warning] too large motion: " << sqrt(dsq)
             << "m, abs(da): " << rad2deg(da) << " deg, this may be a wrong match!" << endl;
        return false;
    }

    // deal with too small motion
    // set 1.5 mm and 0.1 deg transform to 0.
    double zero_trans[3] = {0., 0., 0.};
    double min_dis = params->min_valid_distance;
    double min_angle = deg2rad(params->min_valid_angle);
    if (dsq < min_dis*min_dis && da < min_angle)
        copy_d(zero_trans, 3, delta_trans);

    return true;
}

bool valid_odometry(const sm_params* params, const double* delta_odom)
{
//    double dx = delta_odom[0];
//    double dy = delta_odom[1];
    double da = abs(delta_odom[2]);
//    double dsq = dx * dx + dy * dy;

    // deal with too large odometry
//    double max_dis = params->kf_delta_frame * params->max_linear_velocity * params->laser_frequency;
    double max_angle = params->kf_delta_frame * params->max_angular_velocity * params->laser_frequency;
    if (da > max_angle) {
        cout << "[icp][warning] too large odometry angle: "
             << rad2deg(da) << " deg, this may be a wrong odometry guess!" << endl;
        return false;
    }
//    if ( dsq > max_dis*max_dis || da > max_angle) {
//        cout << "[icp][warning] too large odometry: " << sqrt(dsq)
//             << "m, abs(da): " << rad2deg(da) << " deg, this may be a wrong odometry guess!" << endl;
//        return false;
//    }

    return true;
}

/// @Vance: 设置初始估计，减少迭代次数
void set_first_guess(sm_params* params)
{
    LDP laser_ref  = params->laser_ref;
    LDP laser_sens = params->laser_sens;

    double odometry[3], ominus_laser[3], temp[3];
    pose_diff_d(laser_sens->odometry, laser_ref->odometry, odometry);

    /// @Vance: 如果里程估计出错（如传感器数据发生突变），直接用上一次的角度估计
    if (!valid_odometry(params, odometry))
        copy_d(laser_ref->last_trans, 3, odometry);
    else
        copy_d(laser_ref->last_trans, 2, odometry);

    ominus_d(params->laser, ominus_laser);
    oplus_d(ominus_laser, odometry, temp);
    oplus_d(temp, params->laser, params->first_guess);
}

bool valid_time_stamp(const sm_params* params)
{
    long long ts1, ts2;
    ts1 = params->laser_ref->time_stamp;
    ts2 = params->laser_sens->time_stamp;

    double dts = (ts2 - ts1) * 0.001;
    if (dts > 17 * 1000 * params->laser_frequency) {
        cerr << "[icp][error] Unvalid time stamp for more then 1s (17 frames) !!!\n";
        return false;
    }

    return true;
}
