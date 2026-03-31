# iLQR Planner 故障排除指南

## 常见问题与解决方案

### 1. glog日志错误："COULD NOT CREATE LOGFILE"

**症状：**
```
COULD NOT CREATE LOGFILE '20251211-155700.23678'!
Could not create log file: No such file or directory
```

**原因：**
- glog（Google Logging）库尝试在当前目录创建日志文件
- 在Docker环境或某些目录中可能没有写权限

**解决方案：**
已在代码中配置glog使用`/tmp`目录存储日志：
```cpp
google::InitGoogleLogging(argv[0]);
FLAGS_logtostderr = false;
FLAGS_colorlogtostderr = true;
FLAGS_log_dir = "/tmp";  // 使用/tmp避免权限问题
```

**手动创建日志目录（可选）：**
```bash
mkdir -p ~/.ros/log
chmod 777 ~/.ros/log
```

---

### 2. 仿真器中没有车辆图片

**可能原因：**
1. **规划器节点崩溃** - 检查节点是否正常运行
2. **话题未发布** - 检查ROS话题是否正常
3. **配置文件缺失** - 检查launch文件中的路径配置
4. **时序问题** - 启动顺序不正确

**检查步骤：**

#### Step 1: 检查节点状态
```bash
# 在Docker中执行
docker exec epsilon_planner bash -c "source /root/epsilon/devel/setup.bash && rosnode list"
```

应该看到：
- `/test_ilqr_with_eudm_0` (或类似的规划器节点)
- `/phy_simulator_planning_node`
- 其他相关节点

#### Step 2: 检查话题
```bash
docker exec epsilon_planner bash -c "source /root/epsilon/devel/setup.bash && rostopic list | grep -E '(ctrl|vehicle)'"
```

应该看到：
- `/ctrl/agent_0` - 控制信号
- `/phy_simulator_planning_node/vis/vehicle_set_vis` - 车辆可视化

#### Step 3: 检查话题数据
```bash
docker exec epsilon_planner bash -c "source /root/epsilon/devel/setup.bash && rostopic echo /ctrl/agent_0 -n 1"
```

如果有数据输出，说明规划器正在工作。

#### Step 4: 检查进程
```bash
docker exec epsilon_planner bash -c "ps aux | grep -E '(test_ilqr|test_ssc)' | grep -v grep"
```

不应该看到`<defunct>`（僵尸进程）。

---

### 3. 启动顺序问题

**正确的启动顺序：**

1. **第一个终端 - roscore:**
```bash
roscore
```

2. **第二个终端 - RViz可视化:**
```bash
roscd phy_simulator/rviz/
rviz -d phy_simulator_planning.rviz
```

3. **第三个终端 - 规划器（iLQR或SSC）:**

**iLQR版本:**
```bash
roslaunch planning_integrated test_ilqr_with_eudm_ros.launch
```

**SSC版本:**
```bash
roslaunch planning_integrated test_ssc_with_eudm_ros.launch
```

4. **第四个终端 - AI Agent:**
```bash
roslaunch ai_agent_planner onlane_ai_agent.launch
```

5. **第五个终端 - 物理仿真器:**
```bash
roslaunch phy_simulator phy_simulator_planning.launch
```

**⚠️ 重要：** 必须等每个步骤完全启动后再启动下一个！

---

### 4. 节点崩溃调试

如果看到僵尸进程`<defunct>`：

#### 方法1: 查看ROS日志
```bash
docker exec epsilon_planner bash -c "rosnode list"
docker exec epsilon_planner bash -c "rosnode info /test_ilqr_with_eudm_0"
```

#### 方法2: 直接运行节点查看错误
```bash
docker exec -it epsilon_planner bash
cd /root/epsilon
source devel/setup.bash
rosrun planning_integrated test_ilqr_with_eudm
```

这样可以直接看到错误信息。

#### 方法3: 检查launch文件配置
确保所有必要的参数都已配置：
- `ego_id`
- `agent_config_path`
- `bp_config_path`
- `ilqr_config_path`（可选）

---

### 5. RViz中看不到车辆

**检查项：**

1. **在RViz中添加显示项：**
   - Add → MarkerArray
   - Topic: `/phy_simulator_planning_node/vis/vehicle_set_vis`

2. **检查Fixed Frame：**
   - 应该设置为 `map`

3. **检查仿真器是否发布数据：**
```bash
rostopic hz /phy_simulator_planning_node/vis/vehicle_set_vis
```

4. **检查TF变换：**
```bash
rosrun tf view_frames
```

---

### 6. 编译错误

#### fmt库版本问题
如果遇到`fmt::color`或`fmt::format`错误：
- altro-cpp源码已经修改以兼容旧版fmt
- 确保重新编译：`catkin_make --pkg ilqr_planner`

#### C++标准问题
- `planning_integrated`需要C++14
- 已在CMakeLists.txt中设置：`set(CMAKE_CXX_STANDARD 14)`

#### altro-cpp路径问题
- altro-cpp现在位于：`/epsilon/src/EPSILON/core/common/thirdparty/altro-cpp`
- 如果路径不对，更新`ilqr_planner/CMakeLists.txt`中的`ALTRO_CPP_DIR`

---

### 7. Docker环境问题

#### 进入Docker容器
```bash
docker exec -it epsilon_planner bash
```

#### 检查环境
```bash
source /root/epsilon/devel/setup.bash
env | grep ROS
```

#### 清理并重新编译
```bash
cd /root/epsilon
rm -rf build devel
catkin_make
```

---

### 8. 性能问题

如果规划速度慢：

**调整iLQR参数：**
编辑 `ilqr_server_ros.cpp`:
```cpp
// 减少迭代次数
planner_config_.max_outer_iterations = 10;  // 默认20
planner_config_.max_inner_iterations = 50;   // 默认100

// 调整收敛阈值
planner_config_.cost_tolerance = 1e-3;      // 默认1e-4
planner_config_.constraint_tolerance = 1e-3; // 默认1e-4

// 减少规划时域
planner_config_.time_horizon = 3.0;         // 默认5.0
```

**重新编译：**
```bash
catkin_make --pkg ilqr_planner planning_integrated
```

---

## 快速诊断命令

```bash
# 一键检查所有关键信息
docker exec epsilon_planner bash -c "
source /root/epsilon/devel/setup.bash && 
echo '=== ROS节点 ===' &&
rosnode list &&
echo '' &&
echo '=== 关键话题 ===' &&
rostopic list | grep -E '(ctrl|vehicle|traj)' &&
echo '' &&
echo '=== 进程状态 ===' &&
ps aux | grep -E '(test_ilqr|test_ssc|eudm)' | grep -v grep
"
```

---

## 获取帮助

如果以上方法都不能解决问题：

1. **收集日志：**
```bash
docker exec epsilon_planner bash -c "
rosnode list > /tmp/nodes.log &&
rostopic list > /tmp/topics.log &&
ps aux > /tmp/processes.log
"
```

2. **检查详细错误：**
   - 查看终端输出
   - 检查ROS日志：`~/.ros/log/latest/`
   - 检查系统日志：`dmesg | tail`

3. **对比SSC版本：**
   - 如果SSC版本正常工作，说明环境配置正确
   - 问题可能在iLQR特定代码中

---

**最后更新：** 2024-12-11  
**状态：** glog日志问题已修复，编译通过 ✅

