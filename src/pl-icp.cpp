#include "csm/csm_all.h"
#include "csm/utils.h"
#include "csm/laser_data_yogo.h"
#include "csm/sm_config.h"
#include "icp/icp_yogo.h"
#include "icp/icp_optimization.h"


#include <iostream>
#include <fstream>
#include <sstream>
#include <dirent.h> // for files calculation
#include <chrono>   // for time calculation
#include <iomanip>  // for setprecision

using namespace std;

ofstream fout;
string config_file = "../cfg/sm_config.txt";

int get_num_files(const char* path);
void write_trajectory(const vector<vector<double>> poses, const string& file_name);


int main(int argc, const char*argv[]) {
    if (argc <= 2) {
        cerr << "[icp] Usage: pl-icp data_path trajectory_file" << endl;
        return -1;
    }
    string data_path = argv[1];
    string trajectory_file = argv[2];

    struct sm_params params;
    struct sm_result result;
    struct sm_config config;

    vector<vector<double>> all_poses;
    vector<string> files;

    /// @Vance: 设置参数
    config.readParams(config_file);
    set_params(&params, &config);

    /// @Vance: 从数据集文件中生成激光数据
    int nfiles = get_num_files(data_path.c_str());
    for (int i=0; i<nfiles; ++i) {
      std::stringstream ss;
      ss << data_path << i << ".txt";
      files.push_back(ss.str());
    }
    cout<< "[icp][info] Find tatal " << nfiles << " files in dataset " << data_path << endl;

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
    double first_pose[3] = {0., 0., 0.};
    copy_d(first_pose, 3, laser_ref->true_pose);
    copy_d(first_pose, 3, laser_ref->last_trans);
    /// @Vance: 在这里计算首帧点的笛卡尔坐标，后面icp函数里就不用重复计算了
    ld_compute_cartesian(laser_ref);

    /// @Vance: 为容器添加首个元素
    all_poses.reserve(nfiles);   // distribute space
    vector<double> pose1(first_pose, first_pose+3);
    all_poses.push_back(pose1);

    /// @Vance: 声明相关参数
    unsigned long frame_id = 0;    // 60 ms一帧，可以跑8年
    bool need_new_frame = true;
    int skip_frame = 0, drop_frame = 0, tatal_iter = 0;
    int kf_delta_frame = params.kf_delta_frame;

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

        LDP laser_sens = ld_alloc_new(nrays);
        if (!ld_from_yogo_stream(laser_sens, files[i].c_str(), nrays, frame_id)) {
            cerr << "[icp][error] Could not read scan #" << frame_id << endl;
            return -1;
        }
        if (!ld_valid_fields(laser_sens, &params))  {
            cerr << "[icp][error] Invalid laser data in #" << frame_id << endl;
            return -1;
        }

        /// @Vance: 设置初始估计，减少迭代次数
        params.laser_ref  = laser_ref;
        params.laser_sens = laser_sens;

        if (!valid_time_stamp(&params))
            return -2;

        set_first_guess(&params);

        /// @Vance: 做icp运算
        sm_icp(&params, &result);

        /// @Vance: 检查icp结果
        if(!result.valid){
            cerr << "[icp][error] ICP result unvalid! Set this trans to last one #" << frame_id-1 << endl;
            copy_d(laser_ref->last_trans, 3, result.x);
            if (params.use_imu_yaw_as_absolute_rotation) {
                double delta_o =laser_sens->odometry[2] - laser_ref->odometry[2];
                result.x[2] = delta_o;
            }
            copy_d(result.x, 3, laser_sens->last_trans);
            double gp[3];
            get_global_pose(gp, laser_ref->true_pose, result.x);
            copy_d(gp, 3, laser_sens->true_pose);
            vector<double> tmp_pose(gp, gp+3);
            all_poses.push_back(tmp_pose);

            tatal_iter += result.iterations;
            cout << fixed << std::setprecision(6);
            cout << "[icp][info] #" << frame_id << "# valid transform: " << result.x[0]
                 << ", " << result.x[1] << ", " << result.x[2]
                 << ", iterations = " << result.iterations
                 << ", nvalid = " << result.nvalid << endl;
            cout << "[icp][info] #" << frame_id << "# global pose: " << gp[0]
                 << ", " << gp[1] << ", " << gp[2] << endl;

            // free useless data
            ld_free(laser_ref);
            laser_ref = laser_sens;
            laser_sens = nullptr;

            drop_frame++;
            if (drop_frame > 30) {
                cerr << "[icp][error] Drop frame continuously more then 30. ICP lost!\n";

                // write trajectories still
                write_trajectory(all_poses, trajectory_file);
                return -2;
            }

            need_new_frame = true;
        }
        /// @Vance: PL-ICP成功，获取结果
        else {
            drop_frame = 0;
            if (params.use_imu_yaw_as_absolute_rotation) {
                double delta_o =laser_sens->odometry[2] - laser_ref->odometry[2];
//                cout << "[icp][test] valid result.x: " << result.x[0] << ", "
//                     << result.x[1] << ", " << result.x[2] << endl;
//                cout << "[icp][test] delta odometry: " << delta_o << endl;
                result.x[2] = delta_o;
            }
            copy_d(result.x, 3, laser_sens->last_trans);
            double gp[3];
            get_global_pose(gp, laser_ref->true_pose, result.x);
            copy_d(gp, 3, laser_sens->true_pose);
            vector<double> tmp_pose(gp, gp+3);
            all_poses.push_back(tmp_pose);

            tatal_iter += result.iterations;
            cout << fixed << std::setprecision(6);
            cout << "[icp][info] #" << frame_id << "# valid transform: " << result.x[0]
                 << ", " << result.x[1] << ", " << result.x[2]
                 << ", iterations = " << result.iterations
                 << ", nvalid = " << result.nvalid << endl;
            cout << "[icp][info] #" << frame_id << "# global pose: " << gp[0]
                 << ", " << gp[1] << ", " << gp[2] << endl;

            // free useless data
            ld_free(laser_ref);
            laser_ref = laser_sens;
            laser_sens = nullptr;

            // update need_new_frame flag
            need_new_frame = new_keyframe_needed(&params, result.x);
        }
    }
    ld_free(laser_ref);

    auto t2 = chrono::steady_clock::now();
    double dt = chrono::duration<double, ratio<1,1000>>(t2 - t1).count();
    cout << "[icp][info] Done. Average cost time: " << dt/(frame_id+1) <<
            " ms, iteration: " << (float)tatal_iter/(frame_id+1) << " times." <<endl;

    /// write trajectories
    write_trajectory(all_poses, trajectory_file);

    return 0;
}


/** write the icp trajectory to a file */
void write_trajectory(const vector<vector<double>> poses, const string& file_name)
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
        printf("[icp][error] write to result file failed!\n");
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

