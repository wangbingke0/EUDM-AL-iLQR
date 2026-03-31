# iLQR Planner（AL-iLQR + LQR Warm Start）

`ilqr_planner` 是 EPSILON 中的轨迹优化模块：
- 以 EUDM 给出的前向轨迹作为参考线；
- 在笛卡尔状态空间 `[x, y, yaw, v, a, kappa]` 上做 AL-iLQR 优化；
- 使用时空走廊（SSC corridor）作为几何约束；
- 在求解前先做 LQR 热启动，提升收敛速度与稳定性。

## 1. 代码结构（当前版本）

核心流程已拆分为两个独立类：

- `IlqrLqrWarmStart`
  - 头文件：`inc/ilqr_planner/ilqr_lqr_warm_start.h`
  - 实现：`src/ilqr_planner/ilqr_lqr_warm_start.cpp`
  - 作用：构造并 rollout 热启动轨迹，输出初值给 AL-iLQR。

- `IlqrAlIlqrSolver`
  - 头文件：`inc/ilqr_planner/ilqr_al_ilqr_solver.h`
  - 实现：`src/ilqr_planner/ilqr_al_ilqr_solver.cpp`
  - 作用：构建优化问题、添加代价与约束、调用 altro-cpp 求解器。

外层编排：
- `IlqrPlanner`（`ilqr_planner.cpp`）负责数据准备、参考轨迹采样、调用 `RunALiLQROptimization()`。
- `RunALiLQROptimization()` 内部仅创建 `IlqrAlIlqrSolver` 并执行 `Solve()`。

## 2. 使用的求解器

当前 AL-iLQR 使用的是项目内第三方：

`src/EPSILON/core/common/thirdparty/altro-cpp`

在 `CMakeLists.txt` 中以源码方式编译（不是单独动态库链接），包括：
- `altro/augmented_lagrangian/al_solver.cpp`
- `altro/ilqr/*.cpp`
- `altro/problem/*.cpp`

## 3. 优化模型

### 状态与控制
- 状态：`x = [x, y, yaw, v, a, kappa]`
- 控制：`u = [jerk, dkappa]`

### 动力学（连续时间）
- `dx/dt = v * cos(yaw)`
- `dy/dt = v * sin(yaw)`
- `dyaw/dt = v * kappa`
- `dv/dt = a`
- `da/dt = jerk`
- `dkappa/dt = dkappa`

### 主要代价项
- 阶段代价：状态跟踪 + 控制正则
- 终端代价：终点状态跟踪
- 参考轨迹来自 EUDM，并经过时间重采样（默认 `N=50, dt=0.1s`）

### 主要约束
- 控制约束：`jerk`, `dkappa` 上下界
- 状态约束：`v`, `a`, `kappa` 上下界
- 走廊约束：由 SSC corridor 转换后的笛卡尔走廊约束

## 4. 运行流程（推荐）

建议按以下顺序启动：

1. `roscore`
2. RViz
```bash
roscd phy_simulator/rviz/
rviz -d phy_simulator_planning.rviz
```
3. 规划器（iLQR）
```bash
roslaunch planning_integrated test_ilqr_with_eudm_ros.launch
```
4. AI Agent
```bash
roslaunch ai_agent_planner onlane_ai_agent.launch
```
5. 物理仿真器
```bash
roslaunch phy_simulator phy_simulator_planning.launch
```

对照基线（SSC）：
```bash
roslaunch planning_integrated test_ssc_with_eudm_ros.launch
```

## 5. 编译

### Docker（当前常用环境）
容器名示例：`epsilon_planner`，工程路径：`/root/epsilon`，ROS：`melodic`

```bash
docker exec epsilon_planner bash -lc '
  cd /root/epsilon && \
  source /opt/ros/melodic/setup.bash && \
  catkin_make --only-pkg-with-deps ilqr_planner planning_integrated
'
```

### 本地
```bash
cd ~/epsilon
catkin_make --only-pkg-with-deps ilqr_planner planning_integrated
source devel/setup.bash
```

## 6. 常见日志解读

一次成功优化通常会看到：

- `Attempting LQR warm start...`
- `LQR warm start complete... valid=true`
- `AL-iLQR solve time: ... ms`
- `Solver status: 0`
- `Solver converged successfully`

经验上：
- `LQR warm start` 通常 < 1 ms
- `AL-iLQR solve` 常见 3~10 ms（与场景、约束激活程度、迭代轮数有关）

若求解失败，代码会尝试回退到 warm-start 轨迹，日志会提示：
- `Using warm start trajectory as final output due to: ...`

## 7. 关键话题

`IlqrPlannerServer` 使用前缀：`/vis/agent_<ego_id>/ilqr`

可视化话题包括：
- `/vis/agent_<id>/ilqr/cartesian_trajectory`
- `/vis/agent_<id>/ilqr/lqr_warm_start_trajectory`
- `/vis/agent_<id>/ilqr/cartesian_corridor`
- `/vis/agent_<id>/ilqr/frenet_corridor`
- `/vis/agent_<id>/ilqr/reference_trajectory`
- `/vis/agent_<id>/ilqr/reference_lane`
- `/vis/agent_<id>/ilqr/ego_vehicle`
- `/vis/agent_<id>/ilqr/exec_traj`

控制输出：
- 代码内发布 `ctrl`（私有话题）
- 在 `test_ilqr_with_eudm_ros.launch` 中重映射到 `/ctrl/agent_0`

## 8. 调参入口

主要初始化参数在：
- `src/ilqr_planner/ilqr_server_ros.cpp` 的 `IlqrPlannerServer::Init()`

重点参数：
- 轨迹离散：`time_horizon`, `dt`, `num_knot_points`
- 代价权重：`weight_x/y/yaw/v/a/kappa`, `weight_jerk/dkappa`
- 终端权重：`weight_terminal_*`
- 动力学边界：`max_velocity`, `max_acceleration`, `max_curvature`, `max_dkappa`
- 求解器：`max_outer_iterations`, `max_inner_iterations`, `constraint_tolerance`, `cost_tolerance`

## 9. 目录速览

```text
ilqr_planner/
├── inc/ilqr_planner/
│   ├── ilqr_planner.h
│   ├── ilqr_lqr_warm_start.h
│   ├── ilqr_al_ilqr_solver.h
│   ├── cartesian_dynamics.h
│   ├── cartesian_corridor_constraint.h
│   ├── ilqr_server_ros.h
│   └── ilqr_visualizer.h
├── src/ilqr_planner/
│   ├── ilqr_planner.cpp
│   ├── ilqr_lqr_warm_start.cpp
│   ├── ilqr_al_ilqr_solver.cpp
│   ├── cartesian_dynamics.cpp
│   ├── cartesian_corridor_constraint.cpp
│   ├── ilqr_server_ros.cpp
│   └── ilqr_visualizer.cpp
├── launch/ilqr_planner.launch
└── CMakeLists.txt
```

---

如果你希望，我可以下一步再补一节“日志诊断速查表”（例如 `status!=0`、鼓包、尾段偏离、回退触发条件分别怎么定位）。
