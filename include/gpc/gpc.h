/*
// GPC: A library for the solution of General Point Correspondence problems.
// Copyright (C) 2006 Andrea Censi (andrea at censi dot org)
*/

#ifndef H_GENERAL_POINT_CORRESPONDENCE
#define H_GENERAL_POINT_CORRESPONDENCE

#include <vector>

/// @Vance: 匹配点及权重矩阵
struct gpc_corr {
    double p[2];    /// @Vance: laser_sens.points[i].p
    double q[2];    /// @Vance: laser_ref.points[j1].p

    double C[2][2];

    int valid;
};

/**
// This program solves the general point correspondences problem:
// to find a translation $t$ and rotation $\theta$ that minimize
//
//  \sum_k (rot(theta)*c[k].p+t-c[k].q)^T * c[k].C * (rot(theta)*c[k].p+t-c[k].q)
//
// (see the attached documentation for details).
*/


#define TRACE_ALGO 0

/* Set to 1 to force a check that C is positive semidef.
   Note that you will have some numerical errors! */
#define GPC_CHECK_SEMIDEF 0



/**
 * @brief  在icp迭代中由最小二乘算出新的变换
 * @param[in]   K       有效匹配的数量
 * @param[in]   c       有效匹配的数据容器
 * @param[in]   *x0     没看出有用
 * @param[in]   *cov_x0 没看出有用
 * @param[out]  *x_out  icp迭代中由最小二乘算出的新变换估计
 *
 * if c[k].valid is 0, the correspondence is not used
 */
int gpc_solve(int K, const std::vector<gpc_corr>& c,
    const double *x0, const double *cov_x0,
    double *x_new);

/* Some utilities functions */

/** Computes error for a single correspondence */
double gpc_error(const struct gpc_corr*co, const double*x);

double gpc_total_error(const std::vector<gpc_corr>& co, int n, const double*x);

#endif

