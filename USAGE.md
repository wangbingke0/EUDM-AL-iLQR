# EPSILON iLQR Planner 使用指南

## 运行方式对比

### 原SSC版本
```bash
roslaunch planning_integrated test_ssc_with_eudm_ros.launch
```

### 新iLQR版本（推荐）
```bash
roslaunch planning_integrated test_ilqr_with_eudm_ros.launch
```

## 完整运行步骤

### 1. 启动roscore
```bash
roscore
```

### 2. 启动RViz可视化
```bash
roscd phy_simulator/rviz/
rviz -d phy_simulator_planning.rviz
```

### 3. 启动规划器（选择一个）

**选项A: iLQR规划器（新）**
```bash
roslaunch planning_integrated test_ilqr_with_eudm_ros.launch
```

**选项B: SSC规划器（原）**
```bash
roslaunch planning_integrated test_ssc_with_eudm_ros.launch
```

### 4. 启动AI agent
```bash
roslaunch ai_agent_planner onlane_ai_agent.launch
```

### 5. 启动物理仿真器
```bash
roslaunch phy_simulator phy_simulator_planning.launch
```

## iLQR vs SSC 对比

| 特性 | SSC (QP) | iLQR (altro-cpp) |
|------|----------|------------------|
| 优化方法 | 二次规划（QP） | 迭代线性二次调节器（iLQR） |
| 坐标系 | Frenet (s, d) | Cartesian (x, y, yaw, v, a, kappa) |
| 动力学 | 简化运动学 | 完整6状态运动学模型 |
| 约束处理 | 线性约束 | 增广拉格朗日方法（非线性约束） |
| 求解器 | OOQP | altro-cpp |
| C++标准 | C++11 | C++14 |

## launch文件参数

### test_ilqr_with_eudm_ros.launch
```xml
<param name="ego_id" type="int" value="0" />
<param name="desired_vel" value="20.0"/>  <!-- 期望速度 (m/s) -->
<param name="use_sim_state" value="true"/>  <!-- 使用仿真状态 -->
<param name="ilqr_config_path" type="string" value="" />  <!-- 配置文件路径（可选） -->
```

### playground选项
```xml
<arg name="playground" value = "highway_v1.0" />
<!-- <arg name="playground" value = "ring_tiny_v1.0" /> -->
<!-- <arg name="playground" value = "ring_small_v1.0" /> -->
```

## 可视化话题

iLQR规划器发布以下ROS话题：

- `/vis/agent_0/ilqr/traj_vis` - 优化后的轨迹
- `/vis/agent_0/ilqr/corridor_vis` - 时空走廊
- `/vis/agent_0/ilqr/forward_trajs_vis` - 前向轨迹
- `/vis/agent_0/ilqr/ego_fs_vis` - 自车状态
- `/vis/agent_0/ilqr/map_vis` - 地图信息

**注意**：其他车辆轨迹由 `ai_agent_planner` 发布，无需在iLQR规划器中重复可视化。

## 故障排除

### 编译错误
如果遇到编译错误，确保：
1. C++14标准已启用
2. altro-cpp依赖已正确编译
3. fmt库版本兼容

### 运行时错误
- 检查roscore是否已启动
- 确认所有话题已正确发布：`rostopic list`
- 查看日志：`rosnode info /test_ilqr_with_eudm_0`

## 性能调优

编辑 `ilqr_server_ros.cpp` 中的参数：
- `time_horizon`: 规划时间范围
- `dt`: 时间步长
- `max_outer_iterations`: AL-iLQR外层最大迭代次数
- `max_inner_iterations`: iLQR内层最大迭代次数
- `cost_tolerance`: 代价收敛阈值

## 关于altro-cpp位置

**当前位置**: `/home/wbk/epsilon/src/EPSILON/core/common/thirdparty/altro-cpp` ✅

altro-cpp已经移动到thirdparty目录，与其他第三方库（如ooqp、moodycamel等）保持一致的组织结构。

---

**作者**: HKUST Aerial Robotics Group  
**版本**: 2024-12  
**状态**: ✅ 已测试编译通过


roscd aux_tools/src/
python terminal_server.py
