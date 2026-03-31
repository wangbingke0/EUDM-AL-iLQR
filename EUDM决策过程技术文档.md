# EPSILON项目EUDM决策过程技术文档

**版本**: v1.0  
**日期**: 2024年  
**作者**: HKUST Aerial Robotics Group  

---

## 📋 目录

1. [项目概述](#1-项目概述)
2. [EUDM决策架构](#2-eudm决策架构)
3. [决策过程详解](#3-决策过程详解)
4. [输入数据结构](#4-输入数据结构)
5. [决策流程步骤](#5-决策流程步骤)
6. [输出数据结构](#6-输出数据结构)
7. [代码实现详解](#7-代码实现详解)
8. [性能分析](#8-性能分析)
9. [调试与可视化](#9-调试与可视化)

---

## 1. 项目概述

### 1.1 EPSILON系统简介

**EPSILON** (Efficient Planning System for Automated Vehicles in Highly Interactive Environments) 是一个为高交互环境设计的自动驾驶规划系统。

**核心特点**：
- 🎯 **分层决策**：行为决策(EUDM) + 轨迹规划(SSC)
- 🚀 **实时性强**：20Hz运行频率，单次决策<50ms
- 🧠 **不确定性感知**：考虑周围车辆的多模态行为
- 🛡️ **安全保证**：多层安全检查(RSS、碰撞检测)

### 1.2 EUDM决策器定位

```
┌─────────────────────────────────────────────────┐
│              EPSILON Planning System             │
├─────────────────────────────────────────────────┤
│                                                  │
│  ┌──────────────────┐    ┌─────────────────┐   │
│  │  Semantic Map    │───>│  EUDM Behavior  │   │
│  │    Manager       │    │    Planner      │   │
│  └──────────────────┘    └────────┬────────┘   │
│                                    ↓             │
│                          ┌─────────────────┐    │
│                          │   SSC Trajecto  │    │
│                          │     Planner     │    │
│                          └─────────────────┘    │
│                                                  │
└─────────────────────────────────────────────────┘
```

---

## 2. EUDM决策架构

### 2.1 三层架构

```
┌───────────────────────────────────────────────────┐
│  Layer 1: ROS Server (EudmPlannerServer)          │
│  - 周期触发 (20Hz)                                 │
│  - 消息队列管理                                    │
│  - 线程调度                                        │
└────────────────┬──────────────────────────────────┘
                 ↓
┌───────────────────────────────────────────────────┐
│  Layer 2: Manager (EudmManager)                   │
│  - 上下文管理                                      │
│  - 变道状态维护                                    │
│  - 重规划逻辑                                      │
└────────────────┬──────────────────────────────────┘
                 ↓
┌───────────────────────────────────────────────────┐
│  Layer 3: Planner (EudmPlanner)                   │
│  - DCP树搜索                                       │
│  - 多线程仿真                                      │
│  - 代价评估                                        │
└───────────────────────────────────────────────────┘
```

### 2.2 核心模块

| 模块 | 文件 | 职责 |
|-----|------|------|
| **Server** | `eudm_server_ros.cc` | ROS接口、定时触发 |
| **Manager** | `eudm_manager.cc` | 上下文管理、决策协调 |
| **Planner** | `eudm_planner.cc` | 核心决策算法 |
| **DCP Tree** | `dcp_tree.cc` | 动作序列生成 |
| **Map Adapter** | `map_adapter.cc` | 地图接口适配 |

---

## 3. 决策过程详解

### 3.1 总体流程

```
时间轴: 0ms────────────────────────────────50ms
        │                                   │
        ├──> [准备] (5ms)                   │
        │    • 获取感知数据                 │
        │    • 更新约束条件                 │
        │                                   │
        ├──> [树搜索] (1ms)                 │
        │    • 生成27个候选序列              │
        │                                   │
        ├──> [多线程仿真] (30ms)            │
        │    • 并行评估所有序列             │
        │    • 前向仿真5秒                  │
        │    • 计算代价                     │
        │                                   │
        ├──> [评估选择] (5ms)               │
        │    • 找最小代价序列               │
        │    • 上下文重选择                 │
        │                                   │
        └──> [输出结果] (5ms)               │
             • 构造语义行为                 │
             • 发布给SSC                    │
```

### 3.2 DCP树结构

**DCP** = Decision-making with Closed-loop Prediction

```
树配置:
  • 高度: 5层
  • 每层时长: 1秒
  • 总规划时域: 5秒
  • 时间步长: 0.2秒

动作空间:
  纵向: {保持, 加速, 减速}  (3种)
  横向: {保持车道, 左变道, 右变道} (3种)

⚠️ 重要约束：纵向动作在5秒内保持不变！
  • 不会有 [AAMMD] 这种纵向混合序列
  • 只有 [MMMMM], [AAAAA], [DDDDD] 三种纵向模式
  
候选序列数量: 3 × 9 = 27个
  • 3种纵向模式（M/A/D全程保持）
  • 每种纵向模式9种横向变化
```

**树结构示例**：

```
              Root (选择纵向模式)
                     │
        ┌────────────┼────────────┐
        M            A            D     <- 纵向(全程保持同一个)
        │            │            │
        └─9种横向─┘  └─9种横向─┘  └─9种横向─┘

示例序列:
  [MMMMM|KKKKK]: 保持速度+保持车道
  [AAAAA|KLLLLL]: 全程加速+第2秒左变道
  [DDDDD|RRRRR]: 全程减速+立即右变道
  [AAAAA|KKKLL]: 全程加速+第4秒左变道
```

---

## 4. 输入数据结构

### 4.1 核心输入

```cpp
// 1. 语义地图管理器 (SemanticMapManager)
struct SemanticMapManager {
    // 自车信息
    Vehicle ego_vehicle;           // 位置、速度、加速度
    int ego_lane_id;              // 当前车道ID
    
    // 周围车辆
    SemanticVehicleSet key_vehicles;  // 关键车辆集合
    
    // 地图信息
    LaneNet lane_network;         // 车道网络拓扑
    
    // 时间戳
    decimal_t time_stamp;
};

// 2. 语义车辆 (SemanticVehicle)
struct SemanticVehicle {
    Vehicle vehicle;              // 车辆状态
    Lane lane;                    // 所在车道
    LateralBehavior lat_behavior; // 横向行为
    ProbDistOfLatBehaviors lat_probs;  // 行为概率分布
};

// 3. 任务信息 (Task)
struct Task {
    decimal_t user_desired_vel;   // 用户期望速度
    LaneChangeInfo lc_info;       // 变道约束信息
    bool is_under_ctrl;           // 是否在控制中
};

// 4. 变道约束 (LaneChangeInfo)
struct LaneChangeInfo {
    bool forbid_lane_change_left;      // 禁止左变道
    bool forbid_lane_change_right;     // 禁止右变道
    bool left_solid_lane;              // 左侧实线
    bool right_solid_lane;             // 右侧实线
    bool lane_change_left_unsafe_by_occu;   // 占用不安全
    bool lane_change_right_unsafe_by_occu;
    bool recommend_lc_left;            // 推荐左变道
    bool recommend_lc_right;           // 推荐右变道
};
```

### 4.2 自车状态

```cpp
struct State {
    Vec2f vec_position;      // 位置 (x, y)
    decimal_t angle;         // 航向角 θ
    decimal_t velocity;      // 速度 v
    decimal_t acceleration;  // 加速度 a
    decimal_t curvature;     // 曲率 κ
    decimal_t steer;         // 转向角 δ
    decimal_t time_stamp;    // 时间戳
};
```

---

## 5. 决策流程步骤

### 5.1 阶段1：准备 (Prepare)

**时间**: 5ms  
**文件**: `eudm_planner.cc::RunOnce()`

```cpp
// 步骤1.1: 获取自车信息
if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    return kWrongStatus;
}
ego_id_ = ego_vehicle_.id();
time_stamp_ = ego_vehicle_.state().time_stamp;

// 步骤1.2: 获取车道ID
int ego_lane_id_by_pos;
map_itf_->GetEgoLaneIdByPosition(
    std::vector<int>(), &ego_lane_id_by_pos);

// 步骤1.3: 构建RSS参考车道
map_itf_->GetRefLaneForStateByBehavior(
    ego_vehicle_.state(), 
    std::vector<int>(),
    LateralBehavior::kLaneKeeping,
    forward_len=130.0, backward_len=130.0,
    false, &rss_lane_);

// 步骤1.4: 预剪枝不合理序列
pre_deleted_seq_ids_.clear();
for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    // 删除连续左右变道的序列
    if (has_consecutive_opposite_lane_changes(action_seq)) {
        pre_deleted_seq_ids_.insert(i);
    }
}
```

**输出**：
- ✅ 自车状态
- ✅ 车道ID
- ✅ RSS参考车道
- ✅ 预删除序列ID集合

---

### 5.2 阶段2：树搜索与多线程仿真 (RunEudm)

**时间**: 30ms  
**文件**: `eudm_planner.cc::RunEudm()`

#### 5.2.1 获取周围车辆

```cpp
// 获取关键车辆
common::SemanticVehicleSet surrounding_semantic_vehicles;
map_itf_->GetKeySemanticVehicles(&surrounding_semantic_vehicles);

// 转换为仿真Agent
ForwardSimAgentSet surrounding_fsagents;
GetSurroundingForwardSimAgents(
    surrounding_semantic_vehicles, 
    &surrounding_fsagents);
```

#### 5.2.2 生成动作序列

```cpp
auto action_script = dcp_tree_ptr_->action_script();
int n_sequence = action_script.size();  // 27个序列

// 示例序列（3种纵向 × 9种横向）
// action_script[0]  = [MMMMM|KLLLLL]  - 保持+早左变
// action_script[1]  = [MMMMM|KRRRR]   - 保持+早右变
// ...
// action_script[8]  = [MMMMM|KKKKK]   - 保持+保持
// action_script[9]  = [AAAAA|KLLLLL]  - 加速+早左变
// ...
// action_script[26] = [DDDDD|KKKKK]   - 减速+保持
```

#### 5.2.3 多线程并行仿真

```cpp
std::vector<std::thread> thread_set(n_sequence);
PrepareMultiThreadContainers(n_sequence);

// 创建27个线程（每个序列一个）
for (int i = 0; i < n_sequence; ++i) {
    thread_set[i] = std::thread(
        &EudmPlanner::SimulateActionSequence,
        this, ego_vehicle_, surrounding_fsagents,
        action_script[i], i);
}

// 等待所有线程完成
for (int i = 0; i < n_sequence; ++i) {
    thread_set[i].join();
}

// 注意：纵向动作在整个5秒规划时域内保持不变
// 这是为了简化搜索空间并保证行为连贯性
```

**每个线程的工作**：

```
SimulateActionSequence(action_seq[i], seq_id=i)
  │
  ├─> SimulateScenario
  │     │
  │     └─> 逐层仿真 (5层)
  │           ├─> Layer 0: Action[0], t=0-1s
  │           │     ├─ 设置IDM期望速度
  │           │     ├─ 确定目标车道
  │           │     └─ SimulateSingleAction
  │           │           └─> 逐步仿真 (0.2s × 5步)
  │           │                 ├─ EgoAgentForwardSim
  │           │                 │    ├─ Pure Pursuit控制
  │           │                 │    └─ IDM速度控制
  │           │                 ├─ SurroundingAgentForwardSim
  │           │                 ├─ CostFunction (代价计算)
  │           │                 ├─ StrictSafetyCheck (碰撞)
  │           │                 └─ RSS Check (安全)
  │           │
  │           ├─> Layer 1: Action[1], t=1-2s
  │           ├─> Layer 2: Action[2], t=2-3s
  │           ├─> Layer 3: Action[3], t=3-4s
  │           └─> Layer 4: Action[4], t=4-5s
  │
  └─> 输出结果
        ├─ sim_res[i] = 1/0 (成功/失败)
        ├─ risky_res[i] = 1/0 (有风险/安全)
        ├─ progress_cost[i] = [每层代价]
        ├─ final_cost[i] = 总代价
        ├─ forward_trajs[i] = 自车轨迹
        └─ surround_trajs[i] = 周围车辆轨迹
```

#### 5.2.4 代价计算

**代价函数结构**：

```cpp
struct CostStructure {
    // 效率代价
    struct EfficiencyCost {
        decimal_t ego_to_desired_vel;      // 与期望速度差
        decimal_t leading_to_desired_vel;  // 被前车阻挡
    } efficiency;
    
    // 安全代价
    struct SafetyCost {
        decimal_t rss;          // RSS安全距离违反
        decimal_t occu_lane;    // 车道占用
    } safety;
    
    // 导航代价
    struct NavigationCost {
        decimal_t lane_change_preference;  // 变道偏好
    } navigation;
    
    decimal_t weight;  // 时间权重（折扣因子）
    
    // 总代价
    decimal_t ave() const {
        return (efficiency.ave() + safety.ave() + 
                navigation.ave()) * weight;
    }
};
```

**代价计算示例**：

```cpp
// 每层的代价 (折扣因子 = 0.7)
Layer 0: cost × 0.7^0 = cost × 1.0
Layer 1: cost × 0.7^1 = cost × 0.7
Layer 2: cost × 0.7^2 = cost × 0.49
Layer 3: cost × 0.7^3 = cost × 0.343
Layer 4: cost × 0.7^4 = cost × 0.240

Total = Σ(layer_cost_i × 0.7^i)
```

---

### 5.3 阶段3：评估与选择

**时间**: 5ms  
**文件**: `eudm_planner.cc::EvaluateMultiThreadSimResults()`

```cpp
decimal_t min_cost = kInf;
int best_id = 0;

for (int i = 0; i < num_sequences; ++i) {
    // 跳过失败的序列
    if (sim_res_[i] == 0) continue;
    
    // 计算总代价
    decimal_t cost = 0.0;
    EvaluateSinglePolicyTrajs(
        progress_cost_[i], tail_cost_[i], 
        action_seq, &cost);
    
    final_cost_[i] = cost;
    
    // 更新最小代价
    if (cost < min_cost) {
        min_cost = cost;
        best_id = i;
    }
}

winner_id_ = best_id;
winner_score_ = min_cost;
winner_action_seq_ = action_script[best_id];
```

---

### 5.4 阶段4：上下文重选择 (Reselect)

**时间**: 3ms  
**文件**: `eudm_manager.cc::ReselectByContext()`

**目的**：考虑变道上下文约束，可能调整选择结果

```cpp
// 保存快照
Snapshot snapshot;
SaveSnapshot(&snapshot);

// 重选择
ReselectByContext(stamp, snapshot, &processed_winner_id);

// 考虑因素:
// 1. 变道是否完成
// 2. 变道时间约束
// 3. 禁止变道区域
// 4. 在满足约束的序列中选择代价最小的
```

### 5.5 处于变道中的重规划流程（Replan in Lane Change）

在实际运行中，EUDM 不仅会在“尚未变道”时规划一次完整行为序列，还会在**变道过程中不断重规划**，以便根据最新感知/约束决定是继续、调整还是取消变道。这一部分逻辑主要分布在 `eudm_manager.cc::Prepare()`、`GetReplanDesiredAction()` 和 `EudmPlanner::UpdateDcpTree()` 中。

#### 5.5.1 时间轴视角

假设上一次决策在时间戳 `t0` 选中了一个“左变道”的 Winner 序列：

```
时间轴:  t0                          t1                          t2
        │                           │                           │
        ├── 上一周期决策开始        ├── 处于变道中重规划         ├── 变道完成/取消
        │                           │                           │
        │ Winner 序列:             │ 当前执行进度:             │ 车道 ID 已变化 / HMI 取消
        │  Lat: [K, L, L, L, L]    │  time_since_last_plan ∈   │ lc_context_.completed = true
        │  Lon: [A, A, A, A, A]    │  第一段 L 的时间区间      │ 后续重规划以 K 为根
```

- 在 `t0` 时刻：  
  - 通过 `EudmManager::Run()` 选出 Winner 序列，并保存到 `context_.action_seq`。  
  - 同时更新变道上下文 `lc_context_`，标记“变道正在进行中”（`completed = false`）。

- 在 `t1` 时刻（仍处于 Winner 序列的某个 L/R 动作段内）：  
  - `Prepare()` 调用 `GetReplanDesiredAction(stamp, &desired_action)`：  
    - 根据 `current_time - context_.seq_start_time` 找到此刻应该执行的 action 段。  
    - 如果该段的 `lat` 为 `kLaneChangeLeft` 或 `kLaneChangeRight`，则有：
      - `desired_action.lat = L/R`（说明“当前正在变道中”）。  
  - `UpdateLaneChangeContextByTask()` 此时尚未把 `lc_context_.completed` 置为 `true`，因此不会强制改回保持车道：

```cpp
if (lc_context_.completed) {
  desired_action.lat = DcpLatAction::kLaneKeeping;
}
// 未完成变道时，上面这句不会触发，desired_action.lat 仍为 L/R
```

  - 随后，`bp_.UpdateDcpTree(desired_action)` 会把这个 L/R 作为新的“根动作”：

```cpp
// eudm_manager.cc::Prepare()
bp_.UpdateDcpTree(desired_action);

// eudm_planner.cc::UpdateDcpTree()
void EudmPlanner::UpdateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
}
```

  - `DcpTree::GenerateActionScript()` 以“正在执行的变道动作（L/R）”为根，向后展开后续若干层的横向组合：
    - 例如从 `[L, L, L, L, L]` 展开出 `[L, L, K, K, K]`、`[L, K, K, K, K]` 等。  
    - 这时再用 `ClassifyActionSeq()` 解析这些序列，就可能出现  
      “先 L/R、后 K” 的模式，被标记为 `is_cancel_behavior = true`（取消变道）。

- 在 `t2` 时刻：  
  - 若地图检测到车道 ID 已从原车道切换到目标车道，或 HMI/配置判定这次变道被取消/超时，则  
    - `lc_context_.completed` 会被设为 `true`；  
    - 下一周期的 `Prepare()` 会强制把 `desired_action.lat` 置为 `kLaneKeeping`，  
      随后的 DCP 树又回到“以保持车道 K 为根”的规划模式。

#### 5.5.2 关键条件小结

综合上面的时间轴，“处于变道中的重规划”在代码中的触发条件可以概括为：

1. **上一周期 Winner 序列本身包含变道段**  
   - 即 `context_.action_seq` 中存在 `lat = L/R` 的动作段。
2. **当前时间落在某个变道动作段内部**  
   - `GetReplanDesiredAction()` 返回的 `desired_action.lat = L/R`。
3. **这次变道还未被认为“完成/取消”**  
   - `UpdateLaneChangeContextByTask()` 中 `lc_context_.completed == false`，  
   - 所以 `Prepare()` 中不会把 `desired_action.lat` 改回 `kLaneKeeping`。

只有在这三个条件同时满足时，本周期才处于“变道中的重规划”状态，`DcpTree` 的根为 L/R，从而在代码层面真的会出现“先变道，再保持/取消变道”的那些序列和 `is_cancel_behavior` 相关逻辑。

---

## 6. 输出数据结构

### 6.1 决策结果 (Snapshot)

```cpp
struct Snapshot {
    // 时间信息
    decimal_t time_stamp;
    decimal_t plan_state_time_stamp;
    
    // Winner信息
    int original_winner_id;      // 原始最优序列ID
    int processed_winner_id;     // 重选择后的ID
    
    // 所有候选序列的结果 (size=243)
    std::vector<std::vector<DcpAction>> action_script;
    std::vector<int> sim_res;           // 仿真是否成功
    std::vector<int> risky_res;         // 是否有风险
    std::vector<decimal_t> final_cost;  // 总代价
    std::vector<std::string> sim_info;  // 调试信息
    
    // Winner序列的详细信息
    vec_E<vec_E<Vehicle>> forward_trajs;  // 自车预测轨迹
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs;
    
    // 参考车道
    Lane ref_lane;
    State plan_state;
};
```

### 6.2 语义行为 (SemanticBehavior)

**最终输出给SSC规划器**：

```cpp
struct SemanticBehavior {
    // 离散行为
    LateralBehavior lat_behavior;        // 保持/左变道/右变道
    LongitudinalBehavior lon_behavior;   // 保持/加速/减速
    
    // 期望速度
    decimal_t actual_desired_velocity;
    
    // 参考轨迹 (5秒预测)
    vec_E<vec_E<Vehicle>> forward_trajs;
    
    // 参考车道
    Lane ref_lane;
    
    // 周围车辆预测
    vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs;
    
    // 行为序列
    std::vector<LateralBehavior> forward_behaviors;
};
```

### 6.3 输出示例

**场景**：高速公路超车

```
输入状态:
  • 自车速度: 20 m/s
  • 当前车道: 1
  • 前车速度: 15 m/s
  • 左侧车道: 空闲

决策结果:
┌────────────────────────────────────────────┐
│ Winner ID: 15                               │
│ Action Sequence: [AAAAA | KLLLLL]          │
│   纵向: 加速-加速-加速-加速-加速 (全程加速)   │
│   横向: 保持-左变道-左变道-左变道-左变道      │
│ Total Cost: 12.47                           │
│ Is Risky: False                             │
│ Time Cost: 35.2 ms                          │
└────────────────────────────────────────────┘

输出行为:
  • lat_behavior: LaneChangeLeft
  • lon_behavior: Accelerate
  • desired_velocity: 25.0 m/s
  • ref_lane: Lane 0 (目标车道)
  
预测轨迹: (25个点, 0.2s间隔, 5秒总长)
  t=0.0s: (x=0.0,   y=0.0,  v=20.0)
  t=0.2s: (x=4.1,   y=0.1,  v=20.4)
  t=0.4s: (x=8.3,   y=0.3,  v=20.8)
  ...
  t=2.0s: (x=45.0,  y=3.5,  v=25.0) <- 完成变道
  t=4.8s: (x=165.0, y=3.5,  v=25.0)
```

---

## 7. 代码实现详解

### 7.1 主要类结构

```cpp
// 核心决策类
class EudmPlanner : public Planner {
public:
    // 配置
    Cfg cfg_;                    // 配置参数
    DcpTree* dcp_tree_ptr_;      // DCP树
    
    // 地图接口
    EudmPlannerMapItf* map_itf_;
    
    // 自车信息
    int ego_id_;
    Vehicle ego_vehicle_;
    int ego_lane_id_;
    
    // 结果存储 (243个序列)
    int winner_id_;
    decimal_t winner_score_;
    std::vector<DcpAction> winner_action_seq_;
    std::vector<int> sim_res_;
    std::vector<int> risky_res_;
    std::vector<decimal_t> final_cost_;
    vec_E<vec_E<Vehicle>> forward_trajs_;
    
    // 主要方法
    ErrorType RunOnce();         // 主入口
    ErrorType RunEudm();         // 核心决策
    ErrorType SimulateActionSequence(...);  // 仿真
    ErrorType EvaluateMultiThreadSimResults(...);  // 评估
};
```

### 7.2 关键函数调用链

```
EudmPlannerServer::MainThread()
  └─> PlanCycleCallback()
        └─> EudmManager::Run()
              ├─> Prepare()
              ├─> EudmPlanner::RunOnce()
              │     ├─> 获取自车信息
              │     ├─> 预剪枝
              │     └─> RunEudm()
              │           ├─> GetKeySemanticVehicles()
              │           ├─> 创建243个线程
              │           │     └─> SimulateActionSequence()
              │           │           └─> SimulateScenario()
              │           │                 └─> SimulateSingleAction()
              │           │                       ├─> EgoAgentForwardSim()
              │           │                       ├─> SurroundingAgentForwardSim()
              │           │                       └─> CostFunction()
              │           ├─> 等待所有线程
              │           └─> EvaluateMultiThreadSimResults()
              ├─> SaveSnapshot()
              ├─> ReselectByContext()
              └─> ConstructBehavior()
```

### 7.3 性能优化技巧

**1. 多线程并行**
```cpp
// 使用std::thread并行仿真
std::vector<std::thread> thread_set(243);
for (int i = 0; i < 243; ++i) {
    thread_set[i] = std::thread(...);
}
```

**2. 预剪枝**
```cpp
// 删除物理不可行的序列
if (consecutive_opposite_lane_changes) {
    pre_deleted_seq_ids_.insert(i);
}
```

**3. 提前终止**
```cpp
// 碰撞时立即终止该序列
if (is_collision) {
    sim_res_[seq_id] = 0;
    return kWrongStatus;
}
```

---

## 8. 性能分析

### 8.1 时间开销分解

| 阶段 | 时间 | 占比 |
|-----|------|------|
| 准备阶段 | 5 ms | 10% |
| 树搜索 | 1 ms | 2% |
| 多线程仿真 | 30 ms | 70% |
| 评估选择 | 5 ms | 10% |
| 上下文处理 | 3 ms | 6% |
| 结果发布 | 1 ms | 2% |
| **总计** | **45 ms** | **100%** |

### 8.2 计算资源

**CPU使用**：
- 多核并行：243个线程同时运行
- 建议配置：8核以上CPU
- 峰值负载：~80% (仿真阶段)

**内存使用**：
- 每个序列：~50KB (轨迹数据)
- 总内存：243 × 50KB ≈ 12MB
- 加上其他数据：~30MB

### 8.3 可扩展性

**调整参数影响**：

| 参数 | 当前值 | 影响 |
|-----|--------|------|
| 树高度 | 5层 | 序列数 = 3^5 = 243 |
| 如果改为4层 | 4层 | 序列数 = 3^4 = 81 (快3倍) |
| 如果改为6层 | 6层 | 序列数 = 3^6 = 729 (慢3倍) |

---

## 9. 调试与可视化

### 9.1 日志输出

**准备阶段**：
```
[Eudm][Input]Ego plan state (x,y,theta,v,a,k):
  (105.3, 52.1, 0.15, 20.5, 0.2, 0.01) lane id:1
[Eudm][Setup]Desired vel:25.0 sim_time total:5.0
  lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r]:
  0,0,0,0,0,0
```

**仿真结果**：
```
[Eudm][Result]0 [MMMMM|KKKKK][s:1|r:0|c:15.234]
[Eudm][Result][e;s;n;w:
  2.1_0.5;0.0_0.0;0.0;1.0|
  1.8_0.3;0.0_0.0;0.0;0.7|
  1.5_0.2;0.0_0.0;0.0;0.49|
  ...]
  
[Eudm][Result]87 [AAMMD|KLKKK][s:1|r:0|c:12.470]
...
[Eudm][Result]Sim status: 1 with 198 behaviors.
```

**最终结果**：
```
[Eudm]SUCCESS id:87 [AAMMD|KLKKK] 
  cost: 12.470 time cost: 35.2 ms.
[Eudm]original id 87 reselect : 87
```

### 9.2 可视化工具

**RViz可视化内容**：
1. ✅ 所有候选轨迹（半透明）
2. ✅ Winner轨迹（高亮）
3. ✅ 周围车辆预测
4. ✅ RSS安全区域
5. ✅ 参考车道

**Marker颜色编码**：
- 🟢 绿色：安全序列
- 🔴 红色：有风险序列
- 🔵 蓝色：Winner序列
- ⚪ 灰色：失败序列

---

## 10. 附录

### 10.1 配置参数

**eudm_config.pb.txt 关键参数**：

```protobuf
cost {
  effciency {
    ego_lack_speed_to_desired_unit_cost: 0.3
    ego_over_speed_to_desired_unit_cost: 0.03
  }
  safety {
    rss_cost_enable: true
    occu_lane_unit_cost: 0.10
  }
  navigation {
    lane_change_left_unit_cost: 0.015
    lane_change_right_unit_cost: 0.06
  }
  discount_factor: 0.7
}

sim {
  duration {
    layer: 1.0         # 每层时长
    step: 0.2          # 时间步长
    tree_height: 5     # 树高度
  }
  ego {
    lon {
      idm {
        min_spacing: 2.0
        head_time: 1.0
      }
      limit {
        acc: 1.0
        soft_brake: 1.67
      }
    }
  }
  acc_cmd_vel_gap: 10.0    # 加速时速度增量
  dec_cmd_vel_gap: 10.0    # 减速时速度减量
}
```

### 10.2 常见问题

**Q1: 为什么只有27个候选序列？**
- A: 纵向动作在5秒内保持不变（3种模式），每种纵向模式有9种横向变化，所以是3×9=27。这是为了保证行为连贯性并简化搜索空间。

**Q2: 为什么纵向动作不能混合（如AAMMD）？**
- A: 这是设计约束，为了：① 保证行为连贯性 ② 减少搜索空间 ③ 避免频繁切换导致的不稳定。纵向动作通过IDM连续控制实现平滑过渡。

**Q3: 如何提高决策速度？**
- A: ① 减少树高度 ② 增加CPU核心数 ③ 调整预剪枝策略 ④ 减少横向变化的粒度

**Q4: 如何处理动态障碍物？**
- A: 在前向仿真中，周围车辆使用IDM模型预测，考虑其加速、减速行为

**Q5: 决策失败怎么办？**
- A: ① 记录日志 ② 检查地图数据 ③ 降级为保守策略（减速+保持车道）

### 10.3 参考文献

1. Zhang, Lu, et al. "Efficient uncertainty-aware decision-making for automated driving using guided branching." ICRA 2020.

2. Ding, Wenchao, et al. "EPSILON: An Efficient Planning System for Automated Vehicles in Highly Interactive Environments." IEEE TRO 2021.

3. Ding, Wenchao, et al. "Safe trajectory generation for complex urban environments using spatio-temporal semantic corridor." RA-L 2019.

---

## 文档版本历史

| 版本 | 日期 | 修改内容 |
|-----|------|---------|
| v1.0 | 2024 | 初始版本 |

---

**联系方式**：
- Email: lzhangbz@connect.ust.hk, wdingae@connect.ust.hk
- GitHub: https://github.com/HKUST-Aerial-Robotics/EPSILON

**License**: MIT

