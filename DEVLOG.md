## pl-icp 开发记录

### TODO
+ [BUG]带有图优化版运行时会在程序终止时提示意外结束（即可以正常运行至程序末尾，实质没有什么影响）
+ [BUG]双线程运行过程中会造成两个线程上的icp匹配迭代时误差增大或者没有匹配点
+ [BUG]陀螺仪数据（yaw角）会有突然的跳动，后续数据会在跳动后的基础上变动
+ 做一版滑动窗口带优化的icp，此版可以不用跑双线程
+ 将 laser_data 中的每帧激光角度值转存至 sm_params 中（因为对于每帧激光来说角度值都是固定的）
+ 需要添加对imu yaw角的平滑处理，以处理imu数据突然跳变和原地漂移的情况（15分钟原地漂移5度）
+ 坐标系要调整和机器人的一致 (x)↑→(y)
+ 优化代码/算法，减少运行耗时
+ 终止条件加入一个：相对变化|e-e'|/e足够小，另一个是对应点平均距离S_lst/N足够小


### other method
* 用kd树加速搜索，用法向量加速收敛（尤其在初始迭代时）
* 用堆排序排列对应点的距离，本方法用的是快速排序


## 09.03
- 做了一些规范的注释，对项目结构写了一些说明，见REMEAD.md


### 09.01
- 写了一版SVD求解R,t来替换icp循环估计，耗时更长，此路不通；
- 写了一版简化的R,t求解（参照蔡博的matlab版），耗时减少一个数量级，但精度变差，造成每次迭代误差变大，迭代发散，有改进空间。



### 08.31
- 将icp算出的角度变换直接用imu的yaw值之差替换，精确度大幅度提高。PL-ICP的角度估计任旧是硬伤
- icp角度和Y值准，pl-icp角度不准平移差比较多，单纯的效果icp更好，迭代次数平均多0.2次，时间平均多0.2ms. 用imu的Yaw值代替估计的相对变换角度时，用pl-icp更好。
- 上机跑通此版（v0.2.4）


### 08.30
- 将相对变化有效性判断放入icp函数内，无效的icp匹配将会将此帧平滑处理，连续平滑至30帧判定为丢失定位
- 加入了处理带有时间戳数据的代码


### 08.29
- 增加了一些参数，用于有效变换判断（确定变换的上下限），提高代码鲁棒性，减少噪声的影响
- 在icp-loop内增加了写下点云文件的代码，利用python进行可视化，方便后期查bug


### 08.28
- [BUG FIXED]解决了gsl矩阵运算存在作用域问题，当前版本以不依赖gsl,其矩阵核心是Eigen，多线程可以同时执行一个函数
- 初步跑通了双线程（主线程需要按机器人上的激光频率执行），在PC上优化线程最多占用主线程3帧的时间（180ms）
- 解决了多线程图优化后的位姿更新问题
+ 但在双线程运行过程中会造成主线程某两帧间匹配点数减少，进而导致匹配失败



### 08.27
- 测试了双线程的运行，优化线程和主线程可以同时运行，实现了两个线程的切换
+ 但线程锁无法解决gsl矩阵运算作用域的问题


### 08.24
- 动态确定图优化的窗口
- 上机测试


### 08.23
- 为所有边都添加了信息矩阵的权重计算，相邻点的边只依据迭代次数，非相邻点的边依据迭代次数和相对位姿变换
- 图优化前新增额外边时，调整了考虑潜在边的算法，使额外边的数量根据点数自适应调整


### 08.22
- 为图优化建立了一个线程，但只能以阻塞模式运行，不可用并行关系，因为整套系统内的矩阵运算基于gsl，无法同时进行同一个子程序。
- [BUG FIXED]解决了某两帧间算出的位姿有突然大跳动的问题，问题在于匹配效果太差，超过了迭代次数阈值而没有过滤此帧，
  这里在每次ICP结束后加入了一个相对位姿变换的有效性判断


### 08.21
- 带图优化版的代码移植至机器人上完成
- [BUG FIXED]解决了跑起来的坐标系变换的bug（边的顶点顺序搞错了from/to）
- [BUG FIXED]解决了图优化后位姿不更新的bug，可持续运行不中断

### 08.17
- 添加了g2o至项目中，写了g2o优化函数
- 添加了全局位姿更新判断，距离小于1.5mm且角度小于0.05°视为在原地，不更新全局位姿
- 更新 laser_data 的结构
- 完成了阻塞模式的局部图优化，优化效率70ms以内
- [BUG FIXED]解决了free()结构体出错的Bug，原因在于激光数据比实际情况多了一线，造成LD的理论占用空间与实际大小不匹配

### 08.16
- 8.15前版本放到github上，新建分支，准备做代码优化
- icp的参数改成从配置文件中读取

