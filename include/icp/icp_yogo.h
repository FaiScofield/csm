#ifndef _PL_ICP_
#define _PL_ICP_

#include <vector>

#include "csm/csm_all.h"
#include "csm/utils.h"


/**
 * @brief set_plicp_params 设置icp运行的参数
 * @param[out] params   icp运行的参数
 *
 * 线上使用，此法已经被写到机器人的icp线程里，参数在主线程里设置
 */
void set_plicp_params(sm_params* params);

/**
 * @brief get_global_pose get robot global position
 * @param[out] global_pos   当前帧的全局位姿
 * @param[in] gp_ref        参考帧的全局位姿
 * @param[in] delta_trans   两帧点云间的相对位姿变换
 */
void get_global_pose(double* global_pos, const double* gp_ref, const double* delta_trans);

/**
 * @brief valid_transform   判断相对变换是否正确
 * @param params        相关参数
 * @param delta_trans   相对变换
 * @return  相对变换过大时返回false
 *
 * 相对变换小于阈值时会设为0变换，防止静置漂移
 */
bool valid_transform(sm_params* const params, double* delta_trans);

/**
 * @brief valid_odometry    判断两帧间的里程差值是否合理
 * @param[in] params        相关参数
 * @param[in] delta_odom    两帧间的里程差值
 * @return  里程差值过大时返回false
 *
 * 主要是判断是否有跳帧丢帧和imu数值突变的现象
 */
bool valid_odometry(const sm_params* params, const double* delta_odom);

/**
 * @brief set_first_guess   设置icp初始变换的估计，以减少迭代次数
 * @param[in/out] params    相关参数
 */
void set_first_guess(sm_params* params);

/**
 * @brief valid_time_stamp  判断两帧间的时间戳是否合理
 * @param[in] params    相关参数
 * @return  时间戳相隔1秒一上返回false
 *
 * 主要是判断是否有跳帧丢帧的现象
 */
bool valid_time_stamp(const sm_params* params);

#endif
