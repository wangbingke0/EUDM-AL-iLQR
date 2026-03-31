# GDB 调试指南 - iLQR 内存损坏问题

## 1. 编译调试版本

已修改 `CMakeLists.txt` 使用 `-g -O0` 编译选项（调试符号 + 无优化）。

重新编译：
```bash
cd /root/epsilon
source devel/setup.bash
catkin_make --only-pkg-with-deps ilqr_planner planning_integrated
```

## 2. 启动 GDB 调试

### 方法 1: 使用调试脚本（推荐）
```bash
cd /home/wbk/epsilon
docker exec -it epsilon_planner bash
cd /root/epsilon
source devel/setup.bash
bash /root/epsilon/debug_ilqr.sh
```

### 方法 2: 手动启动 GDB
```bash
# 在 Docker 容器内
cd /root/epsilon
source devel/setup.bash

# 启动 roscore（在另一个终端）
roscore

# 启动 GDB
gdb /root/epsilon/devel/lib/planning_integrated/test_ilqr_with_eudm

# 在 GDB 中设置参数
(gdb) set args ~arena_info_static:=/arena_info_static ~arena_info_dynamic:=/arena_info_dynamic ~ctrl:=/ctrl/agent_0 __name:=test_ilqr_with_eudm_0
(gdb) set environment ROS_MASTER_URI=http://localhost:11311
```

## 3. 关键断点设置

在 GDB 中设置以下断点：

```gdb
# 断点 1: LQR 热启动函数入口
(gdb) break IlqrPlanner::LQRWarmStart

# 断点 2: 保存轨迹前
(gdb) break ilqr_planner.cpp:596

# 断点 3: 访问轨迹状态时
(gdb) break ilqr_planner.cpp:604

# 断点 4: 可视化函数
(gdb) break IlqrVisualizer::VisualizeLQRWarmStartTrajectory

# 断点 5: 轨迹扩展函数
(gdb) break IlqrPlanner::ExtendReferenceTrajectoryToMinimumLength
```

## 4. 运行和检查

```gdb
# 运行程序
(gdb) run

# 当程序停在断点时，检查变量
(gdb) print N
(gdb) print traj
(gdb) print traj->State(0)
(gdb) print lqr_warm_start_traj_.states.size()

# 继续执行
(gdb) continue
```

## 5. 崩溃时检查

当程序崩溃（SIGSEGV）时：

```gdb
# 1. 查看堆栈跟踪
(gdb) bt
(gdb) bt full  # 显示所有局部变量

# 2. 查看当前帧
(gdb) frame
(gdb) info locals
(gdb) info args

# 3. 检查关键变量
(gdb) print N
(gdb) print traj
(gdb) print lqr_warm_start_traj_
(gdb) print reference_cartesian_states_.size()

# 4. 检查内存
(gdb) x/10x $sp  # 查看栈指针附近的内存
(gdb) x/10i $pc  # 查看程序计数器附近的指令

# 5. 检查轨迹状态访问
(gdb) print traj->State(0)
(gdb) print traj->State(N)
(gdb) print traj->Control(0)
```

## 6. 常见问题检查

### 检查 N 的值
```gdb
(gdb) print N
(gdb) print cfg_.num_knot_points
```

### 检查轨迹大小
```gdb
(gdb) print traj->NumStates()
(gdb) print lqr_warm_start_traj_.states.size()
```

### 检查状态有效性
```gdb
(gdb) print traj->State(0)
(gdb) print traj->State(N)
# 检查是否有 NaN 或 Inf
(gdb) print std::isfinite(traj->State(0)(0))
```

### 检查数组边界
```gdb
# 检查是否越界访问
(gdb) print k
(gdb) print N
(gdb) print k <= N
```

## 7. 使用 Valgrind（更详细的检查）

如果 GDB 无法定位问题，可以使用 Valgrind：

```bash
valgrind --leak-check=full \
         --show-leak-kinds=all \
         --track-origins=yes \
         --verbose \
         /root/epsilon/devel/lib/planning_integrated/test_ilqr_with_eudm \
         ~arena_info_static:=/arena_info_static \
         ~arena_info_dynamic:=/arena_info_dynamic \
         ~ctrl:=/ctrl/agent_0 \
         __name:=test_ilqr_with_eudm_0
```

## 8. 调试技巧

1. **逐步执行**：使用 `next` 和 `step` 逐步执行代码
2. **监视变量**：使用 `watch` 监视变量变化
3. **条件断点**：设置条件断点，只在特定条件下停止
   ```gdb
   (gdb) break ilqr_planner.cpp:604 if k > N
   ```
4. **打印调用栈**：使用 `bt` 查看完整的调用栈
5. **检查内存**：使用 `x` 命令检查内存内容

## 9. 可能的问题点

根据代码分析，可能的问题点：

1. **LQRWarmStart 函数**（行 460-612）：
   - 轨迹状态初始化
   - 状态访问 `traj->State(k)`
   - 控制访问 `traj->Control(k)`

2. **保存轨迹**（行 596-680）：
   - 数组大小不匹配
   - 状态复制时的内存访问

3. **可视化函数**（行 300-360）：
   - 访问 `traj.states[k]` 时可能越界
   - 状态有效性检查

4. **轨迹扩展**（行 379-456）：
   - 参考轨迹扩展时的内存分配

