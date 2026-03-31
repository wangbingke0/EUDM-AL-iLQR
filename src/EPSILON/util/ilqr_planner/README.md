# iLQR Planner (Cartesian Coordinates)

基于altro-cpp的增广拉格朗日iLQR（AL-iLQR）轨迹规划器，在笛卡尔坐标系(x, y, t)下进行优化。

## 特性

- **完整的altro-cpp集成**: 使用altro-cpp库的AugmentedLagrangianiLQR求解器
- **笛卡尔坐标系优化**: 在(x, y, t)坐标系下进行轨迹优化
- **自动坐标转换**: 自动将Frenet走廊边界转换为笛卡尔多边形约束
- **6状态车辆模型**: 状态向量 [x, y, yaw, v, a, kappa]
- **2控制输入**: 控制向量 [jerk, dkappa]
- **RViz可视化**: 支持轨迹、走廊、参考线等可视化

## 车辆动力学模型

### 状态向量
```
x = [x, y, yaw, v, a, kappa]
```
- `x`, `y`: 位置 (m)
- `yaw`: 航向角 (rad)
- `v`: 速度 (m/s)
- `a`: 加速度 (m/s²)
- `kappa`: 曲率 (1/m)

### 控制向量
```
u = [jerk, dkappa]
```
- `jerk`: 加速度变化率 (m/s³)
- `dkappa`: 曲率变化率 (1/m/s)

### 动力学方程
```
dx/dt = v * cos(yaw)
dy/dt = v * sin(yaw)
dyaw/dt = v * kappa
dv/dt = a
da/dt = jerk
dkappa/dt = dkappa
```

## 坐标转换

### Frenet → Cartesian 走廊转换
从时空走廊(s, d, t)转换为笛卡尔多边形：
1. 沿走廊边界采样点
2. 使用参考线将(s, d)转换为(x, y)
3. 生成凸多边形表示可行驶区域

### 约束处理
- 使用增广拉格朗日方法处理不等式约束
- 每个knot point的走廊约束独立评估
- 支持控制边界约束

## 使用方法

### 启动节点
```bash
roslaunch ilqr_planner ilqr_planner.launch
```

### 参数配置
- `ego_id`: 自车ID
- `work_rate`: 规划频率 (Hz)
- `config_path`: 配置文件路径

### RViz话题
- `/ilqr_planner/cartesian_trajectory`: 优化后的笛卡尔轨迹
- `/ilqr_planner/cartesian_corridor`: 笛卡尔走廊多边形
- `/ilqr_planner/reference_lane`: 参考线
- `/ilqr_planner/reference_trajectory`: 参考轨迹
- `/ilqr_planner/ego_vehicle`: 自车位置

## 代价函数

### 阶段代价
```
L(x, u) = 0.5 * (x - x_ref)^T * Q * (x - x_ref) + 0.5 * u^T * R * u
```

### 终端代价
```
L_f(x) = 0.5 * (x - x_f)^T * Q_f * (x - x_f)
```

## 求解器配置

### AL-iLQR参数
- `max_iterations_outer`: 外层迭代次数（增广拉格朗日）
- `max_iterations_inner`: 内层迭代次数（iLQR）
- `cost_tolerance`: 代价收敛阈值
- `gradient_tolerance`: 梯度收敛阈值
- `penalty_initial`: 初始惩罚参数
- `penalty_max`: 最大惩罚参数

### 积分器
使用RK4（四阶龙格库塔）方法进行离散化，将连续动力学模型转换为离散时间模型。

## 实现架构

```
ilqr_planner/
├── inc/ilqr_planner/
│   ├── cartesian_dynamics.h           # 笛卡尔车辆动力学模型
│   ├── cartesian_corridor_constraint.h # 走廊约束定义
│   ├── ilqr_planner.h                  # 主规划器类
│   ├── ilqr_visualizer.h               # RViz可视化
│   └── ilqr_server_ros.h               # ROS接口
└── src/ilqr_planner/
    ├── cartesian_dynamics.cpp
    ├── cartesian_corridor_constraint.cpp
    ├── ilqr_planner.cpp
    ├── ilqr_visualizer.cpp
    └── ilqr_server_ros.cpp
```

## 主要改进

相比原始SSC规划器（基于QP）：
1. **非线性优化**: iLQR可以更好地处理非线性车辆动力学
2. **自动微分**: altro-cpp提供自动微分功能
3. **约束处理**: 使用增广拉格朗日方法，比QP更灵活
4. **笛卡尔坐标**: 直接在(x,y,t)空间优化，避免Frenet坐标系的奇异性问题

## 编译问题解决

由于Docker环境中的fmt库版本较老(4.0.0)，我们对altro-cpp源代码进行了以下修改：
- 移除了fmt::color相关功能
- 将fmt::format调用替换为简单的字符串
- 将fmt::print调用替换为std::cout
- 使用Eigen::IOFormat进行矩阵打印

这些修改不影响核心功能，只是移除了一些调试输出的格式化功能。

## 依赖

- ROS (melodic/noetic)
- Eigen3
- altro-cpp (已集成到项目中)
- fmt (v4.0.0+)
- OpenMP

## 编译

```bash
cd ~/epsilon
catkin_make --pkg ilqr_planner
```

## 状态

✅ **编译成功**
✅ **节点运行正常**
✅ **ROS话题已发布**
✅ **完整altro-cpp集成**
✅ **笛卡尔坐标系优化**

## 作者

基于EPSILON项目SSC规划器扩展开发，使用altro-cpp库实现AL-iLQR优化
