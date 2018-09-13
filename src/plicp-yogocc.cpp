#include <iostream>

#include "csm/laser_data_yogo.h"
#include "icp/icp_yogo.h"
#include "icp/icp.h"

using namespace std;

int main(int argc, const char*argv[]) {
    // generate random data for test
    int nrays = 1081;
    int  range_ref[nrays];
    char flags_ref[nrays];
    int  range_curr[nrays];
    char flags_curr[nrays];
    double delta_odom_ref[3] = {0., 0., 0.};
    double delta_odom_curr[3] = {.8, .1, .05};
    srand((unsigned)time(0) );
    for (int i=0; i<nrays; ++i) {
        range_ref[i] = rand()%9000;
        range_curr[i] = range_ref[i] + 1000;
        flags_ref[i] = (char)abs(rand()%2);
        flags_curr[i] = flags_ref[i];
    }
    long long timestamp_ref = 1535638432058660;
    long long timestamp_curr = 1535638432409342;

    sm_params params;
    sm_result result;
    set_plicp_params(&params);

    // input data
    LDP laser_ref = ld_alloc_new(nrays);
    LDP laser_curr = ld_alloc_new(nrays);
    set_laser_frame(laser_ref, range_ref, flags_ref, delta_odom_ref, nrays, 0, timestamp_ref);
    set_laser_frame(laser_curr, range_curr, flags_curr, delta_odom_curr, nrays, 1, timestamp_curr);
    ld_compute_cartesian(laser_ref);

    // check data
    if(!ld_valid_fields(laser_ref, &params))  {
        sm_error(" -icp- Invalid laser data in first scan.\n");
        return -2;
    }
    if(!ld_valid_fields(laser_curr, &params))  {
        sm_error(" -icp- Invalid laser data in second scan.\n");
        return -2;
    }
    if(	any_nan(laser_ref->odometry,3) ||
        any_nan(laser_curr->odometry,3) ) {
        printf("odometry NAN.!!\n");
        return -3;
    }

    params.laser_ref = laser_ref;
    params.laser_sens = laser_curr;
    valid_time_stamp(&params);

    double fg[3];
    pose_diff_d(delta_odom_curr, delta_odom_ref, fg);
    copy_d(fg, 3, params.first_guess);
    printf(" -icp- first guess : [%f, %f, %f]\n", params.first_guess[0], params.first_guess[1], params.first_guess[2]);

    printf(" -icp- set input data sucessed!\n");

    sm_icp(&params, &result);

    // output result
    printf(" -icp- delta transform : [%f, %f, %f]\n", result.x[0], result.x[1], result.x[2]);
    printf(" -icp- iterations: %d\n", result.iterations);

    // free
    ld_free(laser_ref);
    ld_free(laser_curr);

    return 0;
}
