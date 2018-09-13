# pl-icp
A PL-ICP method from the C(canonical) Scan Matcher https://github.com/AndreaCensi/csm




### 工程结构
+ bin
+ cfg           存放配置文件，方便离线调试使用
+ cmake         存放cmake项目模块文件
+ debug_data    存放一些数据集的运行效果文件，有图片和生成的轨迹文档等，对工程本身没有影响
+ lib           存放生成的库文件
+ include / + src
    + csm       工程内数据的格式和基本运算的定义与实现
    + egsl      矩阵运算定义与实现
    + gpc       用于计算icp迭代中的最小二乘问题，求新的变换估计
    + icp       icp的定义与实现
    - pl-icp.cpp                可执行文件，用于读取数据集跑ICP并储存运行轨迹
    - icp_with_optimization.cpp 可执行文件，带有双线程优化版（阻塞版可正常运行，并行版有未知BUG未解决）
    - plicp-yogocc.cpp          可执行文件，用于模拟机器人给出的数据格式的数据读取，线上使用的参考
    - draw_trajectory.py        可执行脚本，用于轨迹绘画
    - draw_iteration.py         可执行脚本，用于可视化icp迭代过程中每一步的点云，用于调试
- DEVLOG        开发记录
    

### 编译及使用方式
#### 离线: 
在Qt下用Release模式编译，运行pl-icp或icp_with_optimization，给出2个参数 —— 数据集路径（要确保路径以“/”结尾）和生成的轨迹文件名，用draw_trajectory.py 可视化查看轨迹.
编译Debug模式时，要将CMakeLists.txt文件中的编译模式设为Debug.
#### 在线：
在Qt下，选择 ARM V7/V8 编译器，用Release模式编译. 编译前要修改的内容:
- 注释掉CMakeLists.txt文件中所有关于g2o的部分
- 注释掉src/CMakeLists.txt文件中所有的可执行文件
- 注释掉src/CMakeLists.txt文件第10行（即csm/sm_config.cpp文件不参与在线的编译）
将"所有头文件"以及"生成的库文件"拷贝到机器人的工程内，并在工程内指定好路径，即可使用

    
    
    
    
    
    
    
    
    
    


