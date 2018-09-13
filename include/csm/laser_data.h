#ifndef H_LASER_DATA
#define H_LASER_DATA

#include <stdio.h>
#include "restrict.h"
#include "algos.h"

struct correspondence;
struct sm_params;

/// @Vance: 存储点的笛卡尔坐标和位姿转换后的极坐标
typedef struct {
    double p[2];
    double rho, phi;        /// @Vance: ρ=sqrt(x*x+y*y), φ=atan2(y,x)
} point2d;

/// @Vance: 存储点云数据的基本结构
struct laser_data {
    int nrays;
    unsigned long frame_id; /// @Vance: for unsigned long and 60ms/frame，it can run 8 years.

    double  min_theta;
    double  max_theta;

    double * restrict theta;
    int    * restrict valid;
    double * restrict readings;

    int    * restrict cluster;

    // Estimated orientation of the surface (radians, relative to robot).
    // It is an estimate of `ld.true_alpha[i]`.
    double * restrict alpha;
    // Estimated covariance of `ld.alpha[i]`.
    double * restrict cov_alpha;
    // True if previous field is valid.
    int    * restrict alpha_valid;
    // Orientation of the normal of the surface (radians, relative to robot).
    // It is `NAN` if not valid.
    double * restrict true_alpha;

    double * restrict readings_sigma;

    struct correspondence*  restrict corr;

    double true_pose[3];
    double odometry[3];
    double estimate[3];
    double last_trans[3];

    /** Cartesian representation */
    point2d *  restrict points;
    /** Cartesian representation, in "world" (laser_ref) coordinates.
        Computed using ld_compute_world_coords() */
    point2d *  restrict points_w;

    /** Timestamp */
    struct timeval tv;      /// @Vance: need to delete
    char hostname[32];      /// @Vance: need to delete
    long long time_stamp;   /// @Vance: true time stamp

    /* Jump tables needed by find_correspondences_tricks(). */
    int * restrict up_bigger,
        * restrict up_smaller,
        * restrict down_bigger,
        * restrict down_smaller;

    /// @Vance: copy constructor
    laser_data(const laser_data& other);
};

/// @Vance: 储存对应点匹配情况的结构
struct correspondence {
    /** 1 if this correspondence is valid  */
    int valid;
    /** Closest point in the other scan.  */
    int j1;
    /** Second closest point in the other scan.  */
    int j2;
    /** Type of correspondence (point to point, or point to line) */
    enum { corr_pp = 0, corr_pl = 1} type;
    /** Squared distance from p(i) to point j1 */
    double dist2_j1;
};

typedef struct laser_data* LDP;

/** This returns a new structure, with all fields initialized */
LDP ld_alloc_new(int nrays);

/** This DOES free() the pointer  */
void ld_free(LDP);

/** This allocs the fields in the given structure. Use ld_alloc_new(), not this. */
void ld_alloc(LDP, int nrays);

/** This does NOT free the pointer. Don't use -- use ld_alloc_new()/ld_free() instead. */
void ld_dealloc(LDP);

/** Fills the x,y fields in "points" by transforming (theta, reading) to cartesian */
void ld_compute_cartesian(LDP);

/** Computes the "points_w" coordinates by roto-translating "points" */
void ld_compute_world_coords(LDP, const double *pose);

/** Fills the fields: *up_bigger, *up_smaller, *down_bigger, *down_smaller.*/
/// @Vance:  其实现在 icp_corr_tricks.cpp 文件里
void ld_create_jump_tables(LDP);

/** Computes an hash of the correspondences */
unsigned int ld_corr_hash(LDP);

/** Returns the number of valid correspondences. */
int ld_num_valid_correspondences(LDP);

/** Do an extensive sanity check about the data contained in the structure. */
int ld_valid_fields(LDP, const sm_params*);

/** A simple clustering algorithm. Sets the `cluster' field in the structure. */
void ld_simple_clustering(LDP ld, double threshold);

/** A cool orientation estimation algorithm. Needs cluster. */
void ld_compute_orientation(LDP ld, int size_neighbourhood, double sigma);


void possible_interval(const double *p_i_w, LDP laser_sens,
                       double max_angular_correction_deg,
                       double max_linear_correction,
                       int*from, int*to, int*start_cell);


#include "laser_data_inline.h"

#endif

