#ifndef _H_ICP_
#define _H_ICP_

//#include <gpc/gpc.h>

#include "csm/csm_all.h"

/**
 * @brief icp计算的接口，返回匹配结果
 * @param[in]   params  输入匹配参数，需要在参数内指定好前后两帧的 laser_data
 * @param[out]  res     输出匹配结果
 */
void sm_icp(struct sm_params*params, struct sm_result*res);

/**
 * @brief icp迭代，返回匹配结果
 * @param[in]   params      输入匹配参数，需要提前做好坐标变换
 * @param[in]   q0          旧的变换估计
 * @param[out]  x_new       新的变换估计
 * @param[out]  total_error 匹配点的总误差
 * @param[out]  nvalid      有效匹配点对数
 * @param[out]  iterations  总迭代次数
 *
 * @return  误差发散时会失败，失败返回0
 */
int icp_loop(struct sm_params*params, const double*q0, double*x_new, 
 	double*total_error, int*nvalid, int*iterations);

/**
 * @brief computing in closed form the next estimate given the correspondences.
 * @param[in]   params      输入匹配参数，需要提前计算好匹配点
 * @param[in]   x_old[3]    旧的变换估计
 * @param[out]  x_new[3]    新的变换估计
 *
 * @rentun 误差发散时会失败，失败返回0
 */
int compute_next_estimate(struct sm_params*params,
	const double x_old[3], double x_new[3]);

/**
 * @brief icp迭代终止条件判断，需要用到epsilon_xy, epsilon_th这个两个参数
 * @param[in]   params  输入匹配参数
 * @param[in]   delta   新旧相对变换之间的差
 *
 * @return 达到终止条件时返回1
 * It is useless when using the point-to-line-distance;
 * however, we put it here because one can choose to use the point-to-point distance.
 */
int termination_criterion(struct sm_params*params, const double*delta);

/** Naif algorithm */
void find_correspondences(struct sm_params*params);
/** Smart algorithm */
void find_correspondences_tricks(struct sm_params*params);
    /** Checks that find_correspondences_tricks and find_correspondences behave the same.
        Exit(-1) on error. */
    void debug_correspondences(struct sm_params * params);

/// @Vance: 匹配点去重与修整（按匹配距离保留一点的百分比）
void kill_outliers_trim(struct sm_params*params, double*total_error);
void kill_outliers_double(struct sm_params*params);

void compute_covariance_exact(
    LDP laser_ref, LDP laser_sens, const gsl_vector*x,
    val *cov0_x, val *dx_dy1, val *dx_dy2);

void visibilityTest(LDP ld, const gsl_vector*x_old);

/** Marks a ray invalid if reading is outside range [min_reading, max_reading]. */
void ld_invalid_if_outside(LDP ld, double min_reading, double max_reading);

void swap_double(double*a,double*b);

/// @Vance: 测试算法，暂时无用
int compute_next_estimate_SVD(struct sm_params* params, double x_new[3]);

/// @Vance: 测试算法，暂时无用
int compute_next_estimate_cai(struct sm_params* params, double x_new[3]);


#endif
