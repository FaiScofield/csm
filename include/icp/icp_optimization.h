#ifndef ICP_OPTIMIZATION_H
#define ICP_OPTIMIZATION_H

#include "csm/csm_all.h"
#include "icp/icp.h"
#include "csm/laser_data_yogo.h"
#include <vector>

/// @Vance: 图优化用的点
struct icp_vertex {
    int vertex_id;              // local
    unsigned long keyframe_id;  // global
    double vertex_pose[3];
};

/// @Vance: 图优化用的边
struct icp_edge {
    int edge_id_to, edge_id_from;
    double transform[3];
    double information[3]; // main (3) value of information matrix (6)
};

/**
 * @brief add_extra_edges 图优化前要新增一些额外的边
 * @param[in]       params      相关参数
 * @param[in/out]   edges       图优化用的边容器
 * @param[in]       keyframes   关键帧数据容器
 * @param[in]       begin_index 关键帧里的优化开始索引
 * @param[in]       last_index  关键帧里的优化结束索引
 * @return  返回新增边的条数
 */
int add_extra_edges(sm_params* params, std::vector<icp_edge>& edges,
                    const std::vector<laser_data>& keyframes,
                    const unsigned long begin_index, const unsigned long last_index);

/**
 * @brief compute_weight_new_edge 计算新增边的权重
 * @param[in]   gp_i        第i帧的全局坐标
 * @param[in]   gp_j        第j帧的全局坐标
 * @param[in]   trans       第i帧到第j帧的icp相对变换
 * @param[in]   iterations  第i帧到第j帧的icp迭代次数
 * @return 返回权值(0~1)
 * 用信息矩阵+权值的方法做优化有问题，实质是icp内部的“自嗨”，评价方案有待需改
 */
double compute_weight_new_edge(const double* gp_i, const double* gp_j,
                               const double* trans, const int iterations);

/**
 * @brief compute_weight_neighbor_edge 计算相邻帧边的权重
 * @param iterations    相邻帧的icp迭代次数
 * @return  返回权值(0~1)
 * 评价方案有待需改
 */
double compute_weight_neighbor_edge(const int iterations);

/**
 * @brief new_keyframe_needed 是否需要跳帧运行icp的判断
 * @param[in] param 相关参数
 * @param[in] trans 相邻帧的icp相对变换
 * @return  相对变换较小且连续跳帧次数未达上限时返回true
 * 此内容可丢弃，为了保证精度，还是尽量每帧都处理
 */
bool new_keyframe_needed(const sm_params* param, const double* trans);

/**
 * @brief need_optimization 是否需要优化的判断
 * @param[in] params        相关参数
 * @param[in] keyframes     关键帧数据容器
 * @param[in] begin_index   上次参与优化时的最后一帧id
 * @param[in] last_index    当前帧id
 * @return  达到优化条件时返回true
 */
bool need_optimization(const sm_params* params, const std::vector<laser_data>& keyframes,
                       const unsigned long begin_index, const unsigned long last_index);

/**
 * @brief write_to_g2o_file 将图优化中的点和边写到g2o文件中
 * @param[in] vertexs   点
 * @param[in] edges     边
 * @param[in] filename  文件名
 * 线上无用
 */
void write_to_g2o_file(const std::vector<icp_vertex>& vertexs, const std::vector<icp_edge>& edges,
                       const std::string filename);

#endif // ICP_OPTIMIZATION_H
