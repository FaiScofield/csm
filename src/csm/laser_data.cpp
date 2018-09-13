#include "csm/csm_all.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>



double* alloc_double_array(int n, double def);
int* alloc_int_array(int n, int def);

/* -------------------------------------------------- */

LDP ld_alloc_new(int nrays) {
    LDP ld = (LDP) malloc(sizeof(struct laser_data));
    ld_alloc(ld, nrays);
    return ld;
}

double* alloc_double_array(int n, double def) {
    double *v = (double*) malloc(sizeof(double)*n);
    int i=0; for(i=0; i<n; ++i) {
        v[i] = def;
    }
    return v;
}

int* alloc_int_array(int n, int def) {
	int *v = (int*) malloc(sizeof(int)*n);
    int i=0; for(i=0; i<n; ++i) {
		v[i] = def;
	}
	return v;
}

void ld_alloc(LDP ld, int nrays) {
    ld->nrays = nrays;

    ld->valid        = alloc_int_array(nrays, 0);
    ld->readings     = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
    ld->readings_sigma = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
    ld->theta        = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());

    ld->min_theta = std::numeric_limits<double>::quiet_NaN();
    ld->max_theta = std::numeric_limits<double>::quiet_NaN();

    ld->cluster      = alloc_int_array(nrays, -1);
    ld->alpha        = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
    ld->cov_alpha    = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());
    ld->alpha_valid  = alloc_int_array(nrays, 0);

    ld->true_alpha   = alloc_double_array(nrays, std::numeric_limits<double>::quiet_NaN());

    ld->up_bigger    = alloc_int_array(nrays, 0);
    ld->up_smaller   = alloc_int_array(nrays, 0);
    ld->down_bigger  = alloc_int_array(nrays, 0);
    ld->down_smaller = alloc_int_array(nrays, 0);

    ld->corr = (struct correspondence*)
        malloc(sizeof(struct correspondence)*nrays);

    int i;
    for(i=0;i<ld->nrays;i++) {
        ld->corr[i].valid = 0;
        ld->corr[i].j1 = -1;
        ld->corr[i].j2 = -1;
    }

    for(i=0;i<3;i++) {
        ld->odometry[i] =
        ld->estimate[i] =
        ld->last_trans[i] =
        ld->true_pose[i] = std::numeric_limits<double>::quiet_NaN();
    }

    ld->points = (point2d*) malloc(nrays * sizeof(point2d));
    ld->points_w = (point2d*) malloc(nrays * sizeof(point2d));

    for(i=0;i<nrays;i++) {
        ld->points[i].p[0] =
        ld->points[i].p[1] =
        ld->points[i].rho =
        ld->points[i].phi = std::numeric_limits<double>::quiet_NaN();
        ld->points_w[i] = ld->points[i];
    }

    strcpy(ld->hostname, "CSM");
}

void ld_free(LDP ld) {
    ld_dealloc(ld);
    if (ld) free(ld);
}

void ld_dealloc(LDP ld){
    if (ld->valid != NULL)          { free(ld->valid);      ld->valid = nullptr; }
    if (ld->readings != NULL)       { free(ld->readings);   ld->readings = nullptr; }
    if (ld->theta != NULL)          { free(ld->theta);      ld->theta = nullptr; }
    if (ld->cluster != NULL)        { free(ld->cluster);    ld->cluster = nullptr; }
    if (ld->alpha != NULL)          { free(ld->alpha);      ld->alpha = nullptr; }
    if (ld->true_alpha != NULL)     { free(ld->true_alpha); ld->true_alpha = nullptr; }
    if (ld->cov_alpha != NULL)      { free(ld->cov_alpha);  ld->cov_alpha = nullptr; }
    if (ld->up_bigger != NULL)      { free(ld->up_bigger);  ld->up_bigger = nullptr; }
    if (ld->up_smaller != NULL)     { free(ld->up_smaller); ld->up_smaller = nullptr; }
    if (ld->corr != NULL)           { free(ld->corr);       ld->corr = nullptr; }
    if (ld->points != NULL)         { free(ld->points);     ld->points = nullptr; }
    if (ld->points_w != NULL)       { free(ld->points_w);   ld->points_w = nullptr; }
    if (ld->alpha_valid != NULL)    { free(ld->alpha_valid);    ld->alpha_valid = nullptr; }
    if (ld->down_bigger != NULL)    { free(ld->down_bigger);    ld->down_bigger = nullptr; }
    if (ld->down_smaller != NULL)   { free(ld->down_smaller);   ld->down_smaller = nullptr; }
    if (ld->readings_sigma != NULL) { free(ld->readings_sigma); ld->readings_sigma = nullptr; }
}


void ld_compute_cartesian(LDP ld) {
    for(int i=0; i<ld->nrays; i++) {
        if(!ld_valid_ray(ld, i)) continue;
        double x = cos(ld->theta[i]) * ld->readings[i];
        double y = sin(ld->theta[i]) * ld->readings[i];

        ld->points[i].p[0] = x,
        ld->points[i].p[1] = y;
        ld->points[i].rho = std::numeric_limits<double>::quiet_NaN();
        ld->points[i].phi = std::numeric_limits<double>::quiet_NaN();
    }
}

/// @Vance: 实际上是根据相对变换转到相对帧坐标系下
void ld_compute_world_coords(LDP ld, const double *pose) {
    double pose_x = pose[0];
    double pose_y = pose[1];
    double pose_theta = pose[2];
    double cos_theta = cos(pose_theta);
    double sin_theta = sin(pose_theta);
    const int nrays = ld->nrays ;

    point2d * points = ld->points;
    point2d * points_w = ld->points_w;
    for(int i=0; i<nrays; ++i) {
        if(!ld_valid_ray(ld,i)) continue;
        double x0 = points[i].p[0],
               y0 = points[i].p[1];

        if(is_nan(x0) || is_nan(y0)) {
            sm_error("ld_compute_world_coords(): I expected that cartesian coords were already computed: ray #%d: %f %f.\n", i, x0, y0);
        }

        double x = cos_theta * x0 - sin_theta * y0 + pose_x;
        double y = sin_theta * x0 + cos_theta * y0 + pose_y;
        points_w[i].p[0] = x;
        points_w[i].p[1] = y;
        /* polar coordinates */
        points_w[i].rho = sqrt(x*x + y*y);
        points_w[i].phi = atan2(y, x);
    }
}


int ld_num_valid_correspondences(LDP ld) {
    int i;
    int num = 0;
    for(i=0; i<ld->nrays; i++) {
        if(ld->corr[i].valid)
            num++;
    }
    return num;
}

/// min_reading and max_reading need a parameter
int ld_valid_fields(LDP ld, const sm_params* params)  {
    if(!ld) {
        sm_error("NULL pointer given as laser_data*.\n");
        return 0;
    }

    int min_nrays = 10;
    int max_nrays = 10000;  // 10000
    if(ld->frame_id < 0) {
        sm_error("Invalid key frame number: %d\n", ld->frame_id);
        return 0;
    }
    if(ld->nrays < min_nrays || ld->nrays > max_nrays) {
        sm_error("Invalid number of rays: %d\n", ld->nrays);
        return 0;
    }
    if(is_nan(ld->min_theta) || is_nan(ld->max_theta)) {
        sm_error("Invalid min / max theta: min_theta = %f max_theta = %f\n",
            ld->min_theta, ld->max_theta);
        return 0;
    }
    double min_fov = deg2rad(20.0);
    double max_fov = 2.01 * M_PI;
    double fov = ld->max_theta - ld->min_theta;
    if( fov < min_fov || fov > max_fov) {
        sm_error("Strange FOV: %f rad = %f deg \n", fov, rad2deg(fov));
        return 0;
    }
    if(fabs(ld->min_theta - ld->theta[0]) > 1e-6) {
        sm_error("Min_theta (%f) should be theta[0] (%f)\n",
            ld->min_theta, ld->theta[0]);
        return 0;
    }
    if(fabs(ld->max_theta - ld->theta[ld->nrays-1]) > 1e-6) {
        sm_error("Min_theta (%f) should be theta[0] (%f)\n",
            ld->max_theta, ld->theta[ld->nrays-1]);
        return 0;
    }
    /* Check that there are valid rays */
    double min_reading = params->min_reading;
    double max_reading = params->max_reading;
    for(int i=0; i<ld->nrays; i++) {
        if (!ld->valid[i]) continue;

        double th = ld->theta[i];
        double r = ld->readings[i];
        if (is_nan(r) || is_nan(th)) {
            sm_error("Ray #%d: r = %f  theta = %f but valid is %d\n", i, r, th, ld->valid[i]);
            ld->valid[i] = 0;
        }
        if( !(min_reading < r && r < max_reading) )
            ld->valid[i] = 0;

//        if(ld->cluster[i] < -1 ) {
//            sm_error("Ray #%d: Invalid cluster value %d\n.", i, ld->cluster[i]);
//            return 0;
//        }

//        if(!is_nan(ld->readings_sigma[i]) && ld->readings_sigma[i] < 0) {
//            sm_error("Ray #%d: has invalid readings_sigma %f \n", i, ld->readings_sigma[i]);
//            return 0;
//        }

    }
    /* Checks that there is at least 10% valid rays */
    int num_valid   = count_equal(ld->valid, ld->nrays, 1);
    if (num_valid < ld->nrays * 0.10) {
        sm_error("Valid: %d/%d invalid: %d.\n", num_valid, ld->nrays);
        return 0;
    }

    return 1;
}


/** Computes an hash of the correspondences */
unsigned int ld_corr_hash(LDP ld){
    unsigned int hash = 0;
    unsigned int i    = 0;

    for(i = 0; i < (unsigned)ld->nrays; ++i) {
        int str = ld_valid_corr(ld, (int)i) ? (ld->corr[i].j1 + 1000*ld->corr[i].j2) : -1;
        hash ^= ((i & 1) == 0) ? (  (hash <<  7) ^ (str) ^ (hash >> 3)) :
                                 (~((hash << 11) ^ (str) ^ (hash >> 5)));
    }

    return (hash & 0x7FFFFFFF);
}

