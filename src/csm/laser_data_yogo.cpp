#include "csm/csm_all.h"
#include "csm/laser_data_yogo.h"

#include <fstream>
#include <iostream>

using namespace  std;


/** input data */
void set_laser_frame(LDP laser, const int* range, const char* flags, const double* odom,
                     int nrays, const unsigned long frame_id, const long long time_stamp)
{
    double delta_a = M_PI / 720;

    // 数据赋值
    laser->frame_id = frame_id;
    laser->max_theta = 0.5 * (nrays-1) * delta_a;
    laser->min_theta = -laser->max_theta;
    laser->time_stamp = time_stamp;

    for (int i=0; i<nrays; ++i) {
        laser->readings[i] = (double)range[i] * 0.001;   // mm to m
        laser->valid[i] = (int)flags[i];
        laser->theta[i] = laser->min_theta + i * delta_a;
    }

    copy_d(odom, 3, laser->odometry);
    copy_d(odom, 3, laser->estimate);
}

bool ld_from_yogo_stream(LDP ldp, const char* file, const int nrays, const unsigned long frame_id)
{
    double delta_a = M_PI / 720;
    double delta_odom[3];
    int imu_data;
    long long time_stamp;

    // open file
    std::ifstream fin(file);
    if (!fin) {
        sm_error("file %s can not be found!\n", file);
        return false;
    }

    // get first line data
    fin >> delta_odom[0] >> delta_odom[1] >> delta_odom[2] >> imu_data >> time_stamp;
    double yaw = imu_data * 0.01 * M_PI / 180;
    if (yaw > M_PI) yaw -= 2 * M_PI;
    if (yaw < -M_PI) yaw += 2 * M_PI;

    // get ranges data
    double min_angle = -(nrays - 1) * 0.5 * delta_a;
    ldp->max_theta = -min_angle;
    ldp->min_theta = min_angle;
    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = yaw;
    ldp->time_stamp = time_stamp;
    ldp->frame_id = frame_id;


    int range = 0;
    bool flag = 0;
    int nonse_data[2];
    int i = 0;
    while (!fin.eof() && i<nrays) {
        fin >> range >> flag >> nonse_data[0] >> nonse_data[1];
        ldp->readings[i] = range * 0.001;
        ldp->valid[i] = flag;
        ldp->theta[i] = min_angle + i * delta_a;

        ++i;
    }

    fin.close();
    return true;
}

void ld_from_keyframe(LDP ldp, const laser_data& laserD)
{
    int nrays = laserD.nrays;

    ldp->nrays = nrays;
    ldp->frame_id = laserD.frame_id;
    ldp->max_theta = laserD.max_theta;
    ldp->min_theta = laserD.min_theta;
    ldp->time_stamp = laserD.time_stamp;

    memcpy(ldp->valid, laserD.valid, sizeof(int)*nrays);
    memcpy(ldp->readings, laserD.readings, sizeof(double)*nrays);
    memcpy(ldp->readings_sigma, laserD.readings_sigma, sizeof(double)*nrays);
    memcpy(ldp->theta, laserD.theta, sizeof(double)*nrays);
    memcpy(ldp->cluster, laserD.cluster, sizeof(int)*nrays);
    memcpy(ldp->alpha, laserD.alpha, sizeof(double)*nrays);
    memcpy(ldp->cov_alpha, laserD.cov_alpha, sizeof(double)*nrays);
    memcpy(ldp->alpha_valid, laserD.alpha_valid, sizeof(int)*nrays);
    memcpy(ldp->true_alpha, laserD.true_alpha, sizeof(double)*nrays);
    memcpy(ldp->up_bigger, laserD.up_bigger, sizeof(int)*nrays);
    memcpy(ldp->up_smaller, laserD.up_smaller, sizeof(int)*nrays);
    memcpy(ldp->down_bigger, laserD.down_bigger, sizeof(int)*nrays);
    memcpy(ldp->down_smaller, laserD.down_smaller, sizeof(int)*nrays);
    memcpy(ldp->corr, laserD.corr, sizeof(struct correspondence)*nrays);

    int i;
    for(i=0; i<nrays; ++i) {
        ldp->corr[i].valid = laserD.corr[i].valid;
        ldp->corr[i].j1 = laserD.corr[i].j1;
        ldp->corr[i].j2 = laserD.corr[i].j2;
        ldp->points[i].p[0] = laserD.points[i].p[0];
        ldp->points[i].p[1] = laserD.points[i].p[1];
        ldp->points[i].rho = laserD.points[i].rho;
        ldp->points[i].phi = laserD.points[i].phi;
        ldp->points_w[i].p[0] = laserD.points_w[i].p[0];
        ldp->points_w[i].p[1] = laserD.points_w[i].p[1];
        ldp->points_w[i].rho = laserD.points_w[i].rho;
        ldp->points_w[i].phi = laserD.points_w[i].phi;
    }
    for(i=0; i<3; ++i) {
        ldp->odometry[i] = laserD.odometry[i];
        ldp->estimate[i] = laserD.estimate[i];
        ldp->true_pose[i] = laserD.true_pose[i];
        ldp->last_trans[i] = laserD.last_trans[i];
    }
}

// copy construct
laser_data::laser_data(const laser_data& other)
{
    ld_alloc(this, other.nrays);

    frame_id = other.frame_id;
    min_theta = other.min_theta;
    max_theta = other.max_theta;
    time_stamp = other.time_stamp;

    memcpy(valid, other.valid, sizeof(int)*nrays);
    memcpy(readings, other.readings, sizeof(double)*nrays);
    memcpy(readings_sigma, other.readings_sigma, sizeof(double)*nrays);
    memcpy(theta, other.theta, sizeof(double)*nrays);
    memcpy(cluster, other.cluster, sizeof(int)*nrays);
    memcpy(alpha, other.alpha, sizeof(double)*nrays);
    memcpy(cov_alpha, other.cov_alpha, sizeof(double)*nrays);
    memcpy(alpha_valid, other.alpha_valid, sizeof(int)*nrays);
    memcpy(true_alpha, other.true_alpha, sizeof(double)*nrays);
    memcpy(up_bigger, other.up_bigger, sizeof(int)*nrays);
    memcpy(up_smaller, other.up_smaller, sizeof(int)*nrays);
    memcpy(down_bigger, other.down_bigger, sizeof(int)*nrays);
    memcpy(down_smaller, other.down_smaller, sizeof(int)*nrays);
    memcpy(corr, other.corr, sizeof(struct correspondence)*nrays);

    int i;
    for(i=0; i<nrays; ++i) {
        corr[i].valid = other.corr[i].valid;
        corr[i].j1 = other.corr[i].j1;
        corr[i].j2 = other.corr[i].j2;
        points[i].p[0] = other.points[i].p[0];
        points[i].p[1] = other.points[i].p[1];
        points[i].rho = other.points[i].rho;
        points[i].phi = other.points[i].phi;
        points_w[i].p[0] = other.points_w[i].p[0];
        points_w[i].p[1] = other.points_w[i].p[1];
        points_w[i].rho = other.points_w[i].rho;
        points_w[i].phi = other.points_w[i].phi;
    }
    for(i=0; i<3; ++i) {
        odometry[i] = other.odometry[i];
        estimate[i] = other.estimate[i];
        true_pose[i] = other.true_pose[i];
        last_trans[i] = other.last_trans[i];
    }
}

// copy construct
sm_params::sm_params(const sm_params* other)
{
    copy_d(other->first_guess, 3, first_guess);
    copy_d(other->laser, 3, laser);

    max_angular_correction_deg = other->max_angular_correction_deg;
    max_linear_correction = other->max_linear_correction;
    max_iterations = other->max_iterations;
    epsilon_xy = other->epsilon_xy;
    epsilon_theta = other->epsilon_theta;
    max_correspondence_dist = other->max_correspondence_dist;
    use_corr_tricks = other->use_corr_tricks;
    restart = other->restart;
    restart_threshold_mean_error = other->restart_threshold_mean_error;
    restart_dt = other->restart_dt;
    restart_dtheta = other->restart_dtheta;
    outliers_maxPerc = other->outliers_maxPerc;
    outliers_adaptive_order = other->outliers_adaptive_order;
    outliers_adaptive_mult = other->outliers_adaptive_mult;
    outliers_remove_doubles = other->outliers_remove_doubles;
    clustering_threshold = other->clustering_threshold;
    orientation_neighbourhood = other->orientation_neighbourhood;
    do_alpha_test = other->do_alpha_test;
    do_alpha_test_thresholdDeg = other->do_alpha_test_thresholdDeg;
    do_visibility_test = other->do_visibility_test;
    use_point_to_line_distance = other->use_point_to_line_distance;
    use_ml_weights = other->use_ml_weights;
    use_sigma_weights = other->use_sigma_weights;
    do_compute_covariance = other->do_compute_covariance;
    debug_verify_tricks = other->debug_verify_tricks;
    sigma = other->sigma;
    min_reading = other->min_reading;
    max_reading = other->max_reading;
    kf_delta_frame = other->kf_delta_frame;
    kf_dist_linear = other->kf_dist_linear;
    kf_dist_angular = other->kf_dist_angular;
    pg_max_iterations = other->pg_max_iterations;
    pg_max_frames = other->pg_max_frames;
    icp_iteration_visible = other->icp_iteration_visible;
    max_angular_velocity = other->max_angular_velocity;
    max_linear_velocity = other->max_linear_velocity;
    min_valid_distance = other->min_valid_distance;
    min_valid_angle = other->min_valid_angle;
    laser_frequency = other->laser_frequency;
    corr_fail_perc = other->corr_fail_perc;
    use_imu_yaw_as_absolute_rotation = other->use_imu_yaw_as_absolute_rotation;
}
