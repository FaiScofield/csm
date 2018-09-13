#include "csm/csm_all.h"
#include "csm/utils.h"
#include "csm/laser_data_yogo.h"
#include "csm/sm_config.h"
#include "icp/icp_yogo.h"
#include "icp/icp_optimization.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>

#include <thread>
#include <mutex>
#include <dirent.h> // for files calculation
#include <chrono>   // for time calculation
#include <iomanip>  // for setprecision

using namespace std;
using namespace g2o;

typedef BlockSolver<BlockSolverTraits<-1,-1> > SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

static mutex mu;
ofstream fout;
string config_file = "../cfg/sm_config.txt";
bool optimization_done = false;
int optimize_time = 0;
double tatal_opt_time = 0.0;

/**
 * @brief get_num_files 获取数据集文件夹内文件数量
 * @param[in] path  数据集路径，注意以"/"结尾
 * @return  返回文件夹内文件数量
 */
int get_num_files(const char* path);

/**
 * @brief write_trajectory  将运动轨迹输出至文件
 * @param[in] poses     每帧的全局坐标容器
 * @param[in] file_name 输出文件名
 */
void write_trajectory(const vector<vector<double>>& poses, const string& file_name);

/**
 * @brief thread_pose_graph 图优化线程函数
 * @param[in]   my_params   相关参数
 * @param[in]   keyframes   关键帧容器
 * @param[out]  vertexs     图优化后更新的节点
 * @param[in]   edges       图优化的边
 * @param[in]   bi          关键帧容器中优化起始索引
 * @param[in]   li          关键帧容器中优化结束索引
 */
void thread_pose_graph(sm_params my_params, vector<laser_data>& keyframes, vector<icp_vertex>& vertexs,
                       vector<icp_edge> edges, const unsigned long bi, const unsigned long li);


int main(int argc, const char*argv[]) {
    if (argc <= 2) {
        cerr << "[icp] Usage: pl_with_optimization data_path trajectory_file" << endl;
        return -1;
    }
    string data_path = argv[1];
    string trajectory_file = argv[2];

    struct sm_params params;
    struct sm_result result;
    struct sm_config config;

    vector<laser_data> keyframes;
    vector<vector<double>> all_poses;
    vector<double> pose_out(3);
    vector<string> files;
    vector<icp_vertex> vertexs;
    vector<icp_edge> edges;
    icp_edge edge;

    /// @Vance: 设置参数
    config.readParams(config_file);
    set_params(&params, &config);

    /// @Vance: 从数据集文件中生成激光数据
    int nfiles = get_num_files(data_path.c_str());
    for (int i=0; i<nfiles; ++i) {
        stringstream ss;
        ss << data_path << i << ".txt";
        files.push_back(ss.str());
    }
    cout << "[icp][info] Find tatal files: " << nfiles << endl;

    /// @Vance: 读取首帧
    int nrays = 1081;
    LDP laser_ref = ld_alloc_new(nrays);
    if (!ld_from_yogo_stream(laser_ref, files[0].c_str(), nrays, 0) ) {
        cerr << "[icp][error] Could not read first scan.\n";
        return -1;
    }
    if (!ld_valid_fields(laser_ref, &params))  {
        cerr << "[icp][error] Invalid laser data in first scan.\n";
        return -1;
    }
    double first_pose[] = {0., 0., 0.};
    copy_d(first_pose, 3, laser_ref->true_pose);
    copy_d(first_pose, 3, laser_ref->last_trans);
    /// @Vance: 在这里计算首帧点的笛卡尔坐标，后面icp函数里就不用重复计算了
    ld_compute_cartesian(laser_ref);

    /// @Vance: 为容器添加首个元素
    vertexs.reserve(params.pg_max_frames);
    edges.reserve(params.pg_max_frames*2);
    all_poses.reserve(nfiles);
    keyframes.reserve(nfiles);
    keyframes.push_back(*laser_ref);    // add first frame

    /// @Vance: 声明相关参数
    unsigned long frame_id = 0, vertex_id = 0;
    bool need_new_frame = true;
    int skip_frame = 0, drop_frame = 0;
    int tatal_iter = 0;
    int kf_delta_frame = params.kf_delta_frame;

    double information[] = {20, 5, 286.4789};
    bool need_opt = false;
    unsigned long opt_begin_index = 0, opt_last_index = 0, tmp_bi = 0;


    /** read data and run icp */
    cout << "[icp][info] Dealting with laser_data ..." << endl;
    auto t1 = std::chrono::steady_clock::now();
    for (int i=1; i<nfiles; ++i) {
        /// @Vance: 是否需要跳帧运行
        if (!need_new_frame && skip_frame < kf_delta_frame) {
            skip_frame++;
            continue;
        }
        frame_id++;
        skip_frame = 0;
        auto frame_start = std::chrono::steady_clock::now();

        LDP laser_sens = ld_alloc_new(nrays);
        if (!ld_from_yogo_stream(laser_sens, files[i].c_str(), nrays, frame_id)) {
            cerr << "[icp][error] Could not read scan #" << frame_id << endl;
            return -1;
        }
        if (!ld_valid_fields(laser_sens, &params))  {
            cerr << "[icp][error] Invalid laser data in #" << frame_id << endl;
            return -1;
        }

        vertex_id++;
        params.laser_ref  = laser_ref;
        params.laser_sens = laser_sens;

        /// @Vance: 设置初始估计，减少迭代次数
        set_first_guess(&params);

        /// @Vance: 做icp运算
        sm_icp(&params, &result);

        /// @Vance: 检查icp结果
        if(!result.valid){
            cerr << "[icp][error] ICP result unvalid! Drop this frame #" << frame_id << endl;
            return -3;
//            vector<double> tmp_pose(3, std::numeric_limits<double>::quiet_NaN());
//            all_poses.push_back(tmp_pose);

//            tatal_iter += result.iterations;
//            ld_free(laser_sens);

//            drop_frame++;
//            if (drop_frame > 30) {
//                cout << "[icp][error] Drop frames reach to 30, which means icp probably lost!/n";

//                /// write trajectories still
//                printf("[icp][info] Write trajectories..\n");
//                write_trajectory(all_poses, trajectory_file);

//                return -2;
//            }

//            need_new_frame = true;
//            continue;
        }
        /// @Vance: PL-ICP成功，获取结果
        else {
            drop_frame = 0;

            if (params.use_imu_yaw_as_absolute_rotation) {
                double delta_o =laser_sens->odometry[2] - laser_ref->odometry[2];
                result.x[2] = delta_o;
            }
            copy_d(result.x, 3, laser_sens->last_trans);
            double gp[3];
            get_global_pose(gp, laser_ref->true_pose, result.x);
            copy_d(gp, 3, laser_sens->true_pose);
            keyframes.push_back(*laser_sens);

            tatal_iter += result.iterations;
            cout << fixed << std::setprecision(6);
            cout << "[icp][info] #" << frame_id << "# valid transform: " << result.x[0]
                 << ", " << result.x[1] << ", " << result.x[2]
                 << ", iterations = " << result.iterations
                 << ", nvalid = " << result.nvalid << endl;
            cout << "[icp][info] #" << frame_id << "# global pose: " << gp[0]
                 << ", " << gp[1] << ", " << gp[2] << endl;

            edge.edge_id_from = vertex_id - 1;
            edge.edge_id_to = vertex_id;
            copy_d(result.x, 3, edge.transform);
            double w = compute_weight_neighbor_edge(result.iterations);
            double inf[3];
            inf[3] = information[0] * w;
            inf[3] = information[1] * w;
            inf[3] = information[2] * w;
            copy_d(inf, 3, edge.information);
            edges.push_back(edge);

            // free useless data
            ld_free(laser_ref);
            laser_ref = laser_sens;
            laser_sens = nullptr;

            // update need_new_frame flag
            need_new_frame = new_keyframe_needed(&params, result.x);
        }

        /// @Vance: 是否开始优化
        need_opt = need_optimization(&params, keyframes, opt_begin_index, frame_id);
        if (need_opt) {
            tmp_bi = opt_begin_index;
            opt_last_index = keyframes.size() - 1;
//            opt_last_index = frame_id;
            opt_begin_index = opt_last_index;

            if (opt_last_index != frame_id)
                cerr << "**** opt_last_index: " << opt_last_index << " != frame_id: " << frame_id << endl;
            sm_params my_params(&params);
            vertexs.clear();
            std::thread thr(thread_pose_graph, my_params, std::ref(keyframes),
                            std::ref(vertexs), std::move(edges),
                            tmp_bi, opt_last_index);
//            thr.detach();   /// @Vance: 并行运行，会使主线程前后两帧匹配误差突然变大[bug]
            thr.join();   /// @Vance: 串行运行，有时正常，有时会卡主（可能是内存满了）
        }
        /// @Vance: 优化结束后更新位姿
        if (optimization_done) {
            cout << "[icp][optimization] Optimization # " << optimize_time << " done. Last index: "
                 << opt_last_index << ", and the frame id now is: " << frame_id << endl;

            // update laser_ref true pose after optimization
            if (opt_last_index - tmp_bi + 1 != vertexs.size())
                cout << "[icp][error] Optimization error for vertexs size != optimization size!\n";
            else for (int i=0; i<vertexs.size(); ++i)
                copy_d(vertexs[i].vertex_pose, 3, keyframes[i+tmp_bi].true_pose);


            // update to current frame
            int n = frame_id - opt_last_index;
            for (int j=0; j<n; ++j) {
                cout << fixed << std::setprecision(6);
                cout << "[icp][optimization] #" << opt_last_index+j+1 << "# global pose used to be: "
                                     << keyframes[opt_last_index+j+1].true_pose[0] << ", "
                                     << keyframes[opt_last_index+j+1].true_pose[1] << ", "
                                     << keyframes[opt_last_index+j+1].true_pose[2] << endl;

                get_global_pose(keyframes[opt_last_index+j+1].true_pose,
                        keyframes[opt_last_index+j].true_pose, keyframes[opt_last_index+j+1].last_trans);

                cout << "[icp][optimization] #" << opt_last_index+j+1 << "# global pose up to: "
                     << keyframes[opt_last_index+j+1].true_pose[0] << ", "
                     << keyframes[opt_last_index+j+1].true_pose[1] << ", "
                     << keyframes[opt_last_index+j+1].true_pose[2] << endl;
            }
            copy_d(keyframes[frame_id].true_pose, 3, laser_ref->true_pose);

            optimize_time++;
            optimization_done = false;
        }

        /// @Vance: 以激光频率16.67Hz运行
        auto frame_end = std::chrono::steady_clock::now();
        double frame_dur = chrono::duration<double, ratio<1,1000>>(frame_end - frame_start).count();
        cout << "[icp][info] #" << frame_id << "# icp done. will sleep for " << 60.0-frame_dur << " ms.\n";
        if (frame_dur < 60.0)
            usleep(1000*(60.0-frame_dur));
        else {
            usleep(1000*(2*60.0-frame_dur));
            cerr << "[icp][warning] ****************** Too much time cost! \n";
        }
    }
    ld_free(laser_ref);

    auto t2 = chrono::steady_clock::now();
    double dt = chrono::duration<double, ratio<1,1000>>(t2 - t1).count();
    cout << "[icp][info] Icp done. Average cost time: " << dt/(frame_id+1) <<
            " ms, iteration: " << (float)tatal_iter/(frame_id+1) << " times." <<endl;
    cout << "[icp][optimization] Optimization tatal cost time: " << tatal_opt_time
         << "ms, average: " << tatal_opt_time/optimize_time << "ms" << endl;

    /// write trajectories
    for (int i=0; i<nfiles; ++i) {
        double gp[3];
        copy_d(keyframes[i].true_pose, 3, gp);
        pose_out = {gp[0], gp[1], gp[2]};
        all_poses.push_back(pose_out);
    }
    write_trajectory(all_poses, trajectory_file);

    return 0;
}


/** for optimization update **/
void thread_pose_graph(sm_params my_params, vector<laser_data>& keyframes, vector<icp_vertex>& vertexs,
                       vector<icp_edge> edges, const unsigned long bi, const unsigned long li)
{
    auto op_start = std::chrono::steady_clock::now();
    optimization_done = false;

    if(mu.try_lock())
        cout << "[icp][optimization] Try to lock thread sucessed." << endl;
    else {
        cerr << "[icp][error] Try to lock thread failed." << endl;
        optimization_done = true;
    }

    int nframes = li - bi + 1;

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmLevenberg* optimizationAlgorithm =
            new OptimizationAlgorithmLevenberg(blockSolver);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(optimizationAlgorithm);
    optimizer.setVerbose(false);

    /// add vertexs
    int nrays = keyframes[0].nrays;
    LDP laser = ld_alloc_new(nrays);
    ld_from_keyframe(laser, keyframes[bi]);
    g2o::VertexSE2* pose = new g2o::VertexSE2;
    pose->setId(0);
    pose->setEstimate(Eigen::Vector3d(laser->true_pose[0], laser->true_pose[1], laser->true_pose[2]));
    pose->setFixed(true);
    optimizer.addVertex(std::move(pose));

    for (size_t i=1; i<nframes; ++i) {
        ld_from_keyframe(laser, keyframes[bi+i]);
        g2o::VertexSE2* pose = new g2o::VertexSE2;
        pose->setId(i);
        pose->setEstimate(Eigen::Vector3d(laser->true_pose[0],
                          laser->true_pose[1], laser->true_pose[2]));
        optimizer.addVertex(std::move(pose));;
    }
    ld_free(laser);

    /// generate new edges
    my_params.kf_delta_frame = 2;
    if (!add_extra_edges(&my_params, edges, keyframes, bi, li)) {
        cerr << "[icp][error] Add zero edges. skip.\n";
        optimization_done = true;
        return;
    }

    /// add edges
    Eigen::Matrix3d martrixEdgetestbai = Eigen::Matrix3d::Identity();
    for (auto& e : edges) {
        martrixEdgetestbai(0,0) = e.information[0];
        martrixEdgetestbai(1,1) = e.information[1];
        martrixEdgetestbai(2,2) = e.information[2];

        g2o::EdgeSE2* icp_edge = new g2o::EdgeSE2;
        icp_edge->vertices()[0] = optimizer.vertex(e.edge_id_from);
        icp_edge->vertices()[1] = optimizer.vertex(e.edge_id_to);
        icp_edge->setMeasurement(Eigen::Vector3d(e.transform[0],
                                 e.transform[1], e.transform[2]));
        icp_edge->setInformation(martrixEdgetestbai);

        optimizer.addEdge(std::move(icp_edge));
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(my_params.pg_max_iterations);

    // update pose
    Eigen::Vector3d es;
    icp_vertex v;
    for(size_t i = 0; i < nframes; ++i) {
        g2o::VertexSE2* vSE2 = static_cast<g2o::VertexSE2*>(optimizer.vertex(i));
        es = vSE2->estimate().toVector();
        v.vertex_id = i;
        v.keyframe_id = i + bi;
        v.vertex_pose[0] = es[0];
        v.vertex_pose[1] = es[1];
        v.vertex_pose[2] = es[2];
        vertexs.push_back(v);
    }

    optimizer.clear();
    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();

    optimization_done = true;
    auto op_end = std::chrono::steady_clock::now();
    double op_dura = chrono::duration<double, ratio<1,1000>>(op_end - op_start).count();
    tatal_opt_time += op_dura;
    cout << "[icp][optimization] Optimization # " << optimize_time
         << " cost time: " << op_dura << "ms" << endl;

    mu.unlock();
}

/** for trajectories wrote to a file **/
void write_trajectory(const vector<vector<double>>& poses, const string& file_name)
{
    try {
        fout.open(file_name.c_str());
        fout.close();
        fout.open(file_name.c_str(), ios::app);
        for (int i=0; i<poses.size(); ++i) {
            fout << poses[i][0] << " "
                 << poses[i][1]  << " "
                 << poses[i][2]  << "\n";
        }
        fout.close();
        cout << "[icp][info] Write to trajectory file sucessed!\n";
    } catch(...) {
        cout << "[icp][error] Write to result file failed!\n";
    }
}

/** for the file number calculation in the dataset */
static int filterDot(const struct dirent * dir) {
  if (strcmp(dir->d_name,".") == 0 || strcmp(dir->d_name, "..") == 0) {
    // 过滤掉 "."和".."
    return 0;
  } else {
    return 1;
  }
}
int get_num_files(const char* path)
{
    struct dirent **namelist;

    int file_nums = scandir(path, &namelist, filterDot, alphasort);

    free(namelist);
    return file_nums;
}


