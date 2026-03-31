# EUDM决策输出数据详解

## 📊 决策输出数据全景图

```
                    EUDM Planner
                         │
                         ↓
            ┌────────────────────────┐
            │   决策结果 (Snapshot)   │
            └────────────────────────┘
                         │
        ┌────────────────┼────────────────┐
        ↓                ↓                ↓
   Winner信息       所有候选结果      详细轨迹数据
        │                │                │
        ↓                ↓                ↓
  ┌──────────┐    ┌──────────┐    ┌──────────┐
  │序列ID    │    │仿真状态  │    │自车轨迹  │
  │动作序列  │    │代价分布  │    │周围轨迹  │
  │总代价    │    │风险标记  │    │行为序列  │
  └──────────┘    └──────────┘    └──────────┘
        │                │                │
        └────────────────┴────────────────┘
                         │
                         ↓
            ┌────────────────────────┐
            │  语义行为 (Behavior)    │
            └────────────────────────┘
                         │
                         ↓
                    SSC Planner
```

---

## 1. Snapshot 快照数据结构

### 1.1 完整定义

```cpp
struct Snapshot {
    // ═══════════════════════════════════════
    // 时间戳信息
    // ═══════════════════════════════════════
    decimal_t time_stamp;              // 决策时刻的时间戳
    decimal_t plan_state_time_stamp;   // 规划状态的时间戳
    
    // ═══════════════════════════════════════
    // Winner 识别信息
    // ═══════════════════════════════════════
    int original_winner_id;            // 代价最小的序列ID
    int processed_winner_id;           // 经过上下文调整后的序列ID
    
    // ═══════════════════════════════════════
    // 所有候选序列的全局结果 (每个数组 size=27)
    // ═══════════════════════════════════════
    std::vector<std::vector<DcpAction>> action_script;
    // 每个序列的动作序列，如 [AAMMD|KLKKK]
    
    std::vector<int> sim_res;
    // 仿真是否成功: 1=成功, 0=失败
    // 失败原因可能: 碰撞、出界、违反约束
    
    std::vector<int> risky_res;
    // 是否有风险: 1=有风险, 0=安全
    // RSS检查或严格安全检查不通过
    
    std::vector<decimal_t> final_cost;
    // 每个序列的总代价 (所有层代价加权和)
    
    std::vector<std::string> sim_info;
    // 仿真调试信息，如 "(Risky) 102 315"
    
    // ═══════════════════════════════════════
    // 每个序列的代价细节
    // ═══════════════════════════════════════
    std::vector<std::vector<CostStructure>> progress_cost;
    // progress_cost[seq_id][layer_id] = 该层的代价结构
    // 每层包含: efficiency, safety, navigation
    
    std::vector<CostStructure> tail_cost;
    // 最后一层的尾部代价
    
    // ═══════════════════════════════════════
    // Winner 序列的详细轨迹
    // ═══════════════════════════════════════
    vec_E<vec_E<common::Vehicle>> forward_trajs;
    // forward_trajs[winner_id] = 自车5秒预测轨迹 (25个点)
    
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    // forward_lat_behaviors[winner_id] = 横向行为序列
    
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    // forward_lon_behaviors[winner_id] = 纵向行为序列
    
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs;
    // surround_trajs[winner_id][vehicle_id] = 周围车辆轨迹
    
    // ═══════════════════════════════════════
    // 规划状态和参考车道
    // ═══════════════════════════════════════
    common::State plan_state;     // 规划时刻的自车状态
    common::Lane ref_lane;        // 参考车道
};
```

### 1.2 数据示例

**场景**：高速公路，自车速度20m/s，准备超车

```json
{
  "time_stamp": 12.345,
  "plan_state_time_stamp": 12.345,
  
  "original_winner_id": 87,
  "processed_winner_id": 87,
  
  "action_script": {
    "0": ["MMMMM", "KLLLLL"],   // 保持+早左变
    "15": ["AAAAA", "KLLLLL"],  // 全程加速+早左变
    "26": ["DDDDD", "KKKKK"]    // 全程减速+保持
  },
  
  "sim_res": [1, 1, ..., 1, 0, ...],  // 27个结果
  "risky_res": [0, 0, ..., 0, 1, ...],
  
  "final_cost": [
    15.234,    // 序列0: 保持+早左变
    ...
    12.470,    // 序列15: 加速+早左变，超车成功，代价最低
    ...
    kInf       // 序列16: 加速+早右变，碰撞，代价无穷大
  ],
  
  "sim_info": [
    "",
    ...
    "",                        // 序列15: 成功且安全
    ...
    "(Collision) 102"          // 序列16: 与车辆102碰撞
  ],
  
  "progress_cost[15]": [
    {  // Layer 0 (0-1s)
      "efficiency": {"ego": 2.1, "leading": 0.5},
      "safety": {"rss": 0.0, "occu": 0.0},
      "navigation": {"lc": 0.0},
      "weight": 1.0,
      "total": 2.6
    },
    {  // Layer 1 (1-2s)
      "efficiency": {"ego": 1.8, "leading": 0.3},
      "safety": {"rss": 0.0, "occu": 0.0},
      "navigation": {"lc": 5.0},  // 开始变道
      "weight": 0.7,
      "total": 4.97
    },
    ...
  ],
  
  "forward_trajs[15]": [
    // 25个时间点的轨迹
    {"t": 0.0, "x": 0.0,   "y": 0.0,  "v": 20.0, "a": 1.0},
    {"t": 0.2, "x": 4.1,   "y": 0.05, "v": 20.2, "a": 1.0},
    {"t": 0.4, "x": 8.3,   "y": 0.15, "v": 20.4, "a": 1.0},
    ...
    {"t": 2.0, "x": 45.0,  "y": 3.5,  "v": 25.0, "a": 0.0},  // 变道完成
    ...
    {"t": 4.8, "x": 165.0, "y": 3.5,  "v": 25.0, "a": 0.0}
  ]
}
```

---

## 2. SemanticBehavior 语义行为输出

### 2.1 完整定义

```cpp
struct SemanticBehavior {
    // ═══════════════════════════════════════
    // 离散行为指令
    // ═══════════════════════════════════════
    common::LateralBehavior lat_behavior;
    // LaneKeeping, LaneChangeLeft, LaneChangeRight
    
    common::LongitudinalBehavior lon_behavior;
    // Maintain, Accelerate, Decelerate
    
    // ═══════════════════════════════════════
    // 速度规划
    // ═══════════════════════════════════════
    decimal_t actual_desired_velocity;
    // IDM期望速度，根据lon_behavior调整
    
    // ═══════════════════════════════════════
    // 参考轨迹 (供SSC规划器使用)
    // ═══════════════════════════════════════
    vec_E<vec_E<common::Vehicle>> forward_trajs;
    // 自车5秒预测轨迹 (粗略参考)
    
    // ═══════════════════════════════════════
    // 参考车道
    // ═══════════════════════════════════════
    common::Lane ref_lane;
    // 目标车道的几何信息
    // 包含: center_line, left_boundary, right_boundary
    
    // ═══════════════════════════════════════
    // 行为序列 (时间展开)
    // ═══════════════════════════════════════
    std::vector<common::LateralBehavior> forward_behaviors;
    // 每个时间步的横向行为
    
    // ═══════════════════════════════════════
    // 周围车辆预测 (交互感知)
    // ═══════════════════════════════════════
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs;
    // surround_trajs[0][vehicle_id] = 该车辆的预测轨迹
    // 用于SSC规划时的障碍物避让
};
```

### 2.2 从Snapshot到SemanticBehavior的转换

```cpp
void EudmManager::ConstructBehavior(
    common::SemanticBehavior* behavior) {
    
    int seq_id = last_snapshot_.processed_winner_id;
    auto action_seq = last_snapshot_.action_script[seq_id];
    
    // 1. 提取离散行为
    decimal_t operation_at_seconds;
    bool is_cancel_behavior;
    ClassifyActionSeq(action_seq, 
                     &operation_at_seconds,
                     &behavior->lat_behavior,
                     &is_cancel_behavior);
    
    // 第一个动作的纵向行为
    behavior->lon_behavior = GetLonBehaviorByAction(
        action_seq[0].lon);
    
    // 2. 期望速度
    behavior->actual_desired_velocity = 
        CalculateDesiredVel(action_seq[0], bp_.desired_velocity());
    
    // 3. 参考轨迹
    behavior->forward_trajs = 
        last_snapshot_.forward_trajs[seq_id];
    
    // 4. 参考车道
    behavior->ref_lane = last_snapshot_.ref_lane;
    
    // 5. 行为序列
    behavior->forward_behaviors = 
        last_snapshot_.forward_lat_behaviors[seq_id];
    
    // 6. 周围车辆预测
    behavior->surround_trajs = 
        last_snapshot_.surround_trajs;
}
```

---

## 3. 详细数据实例

### 3.1 实例1: 保持车道巡航

**场景**：
- 前方无车，道路畅通
- 当前速度接近期望速度

**输入**：
```
ego_velocity: 24.5 m/s
desired_velocity: 25.0 m/s
ego_lane_id: 1
front_vehicle: None
```

**输出Snapshot**：
```cpp
original_winner_id: 0
processed_winner_id: 0
action_script[0]: [MMMMM|KKKKK]

final_cost[0]: 1.05  // 非常小的代价

progress_cost[0][0]: {
    efficiency: {ego: 0.15, leading: 0.0},  // 速度差很小
    safety: {rss: 0.0, occu: 0.0},
    navigation: {lc: 0.0},
    weight: 1.0,
    total: 0.15
}

forward_trajs[0]: [
    {t: 0.0, x: 0.0,   y: 0.0, v: 24.5},
    {t: 0.2, x: 4.91,  y: 0.0, v: 24.6},
    {t: 0.4, x: 9.83,  y: 0.0, v: 24.7},
    {t: 0.6, x: 14.76, y: 0.0, v: 24.8},
    {t: 0.8, x: 19.70, y: 0.0, v: 24.9},
    {t: 1.0, x: 24.65, y: 0.0, v: 25.0},  // 达到期望速度
    ...
]
```

**输出SemanticBehavior**：
```cpp
lat_behavior: LaneKeeping
lon_behavior: Maintain
actual_desired_velocity: 25.0
ref_lane: Lane 1 (center_line: [(0,0), (100,0), ...])
```

---

### 3.2 实例2: 左变道超车

**场景**：
- 前方有慢车 (15 m/s)
- 左侧车道空闲
- 自车期望速度 25 m/s

**输入**：
```
ego_velocity: 20.0 m/s
desired_velocity: 25.0 m/s
ego_lane_id: 1
front_vehicle: {id: 102, v: 15.0, distance: 30.0}
left_lane_id: 0
left_lane_status: Free
```

**输出Snapshot**：
```cpp
original_winner_id: 87
processed_winner_id: 87
action_script[87]: [AAMMD|KLKKK]
// 加速-加速-保持-保持-减速
// 保持-左变道-保持-保持-保持

final_cost[87]: 12.47  // 较低代价（变道有成本）

// 对比其他序列:
final_cost[0]: 18.52   // [MMMMM|KKKKK] 被前车阻挡
final_cost[243]: kInf  // [AAMMD|RRRRR] 右变道不可行

progress_cost[87]:
  Layer 0 (0-1s): {
    efficiency: {ego: 2.1, leading: 0.5},
    safety: {rss: 0.0, occu: 0.0},
    navigation: {lc: 0.0},  // 还未变道
    weight: 1.0,
    total: 2.6
  }
  
  Layer 1 (1-2s): {
    efficiency: {ego: 1.8, leading: 0.3},
    safety: {rss: 0.0, occu: 0.0},
    navigation: {lc: 5.0},  // 变道代价
    weight: 0.7,
    total: (1.8+0.3+0.0+5.0) * 0.7 = 4.97
  }
  
  Layer 2 (2-3s): {
    efficiency: {ego: 0.5, leading: 0.0},  // 超车成功
    safety: {rss: 0.0, occu: 0.0},
    navigation: {lc: 0.0},
    weight: 0.49,
    total: 0.245
  }
  
  ...
  
  Total: 2.6 + 4.97 + 0.245 + ... = 12.47
```

**输出轨迹**：
```cpp
forward_trajs[87]:
  // 第一阶段: 加速准备 (0-1s)
  {t: 0.0, x: 0.0,   y: 0.0,  v: 20.0, lane: 1}
  {t: 0.2, x: 4.1,   y: 0.0,  v: 20.2, lane: 1}
  {t: 0.4, x: 8.3,   y: 0.0,  v: 20.4, lane: 1}
  {t: 0.6, x: 12.5,  y: 0.0,  v: 20.6, lane: 1}
  {t: 0.8, x: 16.8,  y: 0.0,  v: 20.8, lane: 1}
  {t: 1.0, x: 21.0,  y: 0.0,  v: 21.0, lane: 1}
  
  // 第二阶段: 变道执行 (1-2s)
  {t: 1.2, x: 25.3,  y: 0.5,  v: 21.2, lane: 1→0}
  {t: 1.4, x: 29.7,  y: 1.2,  v: 21.4, lane: 1→0}
  {t: 1.6, x: 34.2,  y: 2.0,  v: 21.6, lane: 1→0}
  {t: 1.8, x: 38.7,  y: 2.8,  v: 21.8, lane: 1→0}
  {t: 2.0, x: 43.3,  y: 3.5,  v: 22.0, lane: 0}  // 完成变道
  
  // 第三阶段: 加速巡航 (2-5s)
  {t: 2.2, x: 48.0,  y: 3.5,  v: 22.5, lane: 0}
  {t: 2.4, x: 52.8,  y: 3.5,  v: 23.0, lane: 0}
  ...
  {t: 4.0, x: 105.0, y: 3.5,  v: 25.0, lane: 0}  // 达到期望速度
  {t: 5.0, x: 130.0, y: 3.5,  v: 25.0, lane: 0}

surround_trajs[87][102]:  // 前车102的预测
  {t: 0.0, x: 30.0,  y: 0.0, v: 15.0}
  {t: 1.0, x: 45.0,  y: 0.0, v: 15.0}
  {t: 2.0, x: 60.0,  y: 0.0, v: 15.0}  // 被超越
  {t: 3.0, x: 75.0,  y: 0.0, v: 15.0}
  {t: 5.0, x: 105.0, y: 0.0, v: 15.0}
```

**输出SemanticBehavior**：
```cpp
lat_behavior: LaneChangeLeft
lon_behavior: Accelerate
actual_desired_velocity: 30.0  // 25 + 5 (加速gap)
ref_lane: Lane 0  // 目标车道
forward_behaviors: [
    LaneKeeping,      // 0-1s
    LaneChangeLeft,   // 1-2s
    LaneKeeping,      // 2-3s
    LaneKeeping,      // 3-4s
    LaneKeeping       // 4-5s
]
```

---

### 3.3 实例3: 紧急减速避让

**场景**：
- 前方车辆突然刹车
- 距离过近，无法变道
- 需要紧急减速

**输入**：
```
ego_velocity: 25.0 m/s                                                                                                                                       
front_vehicle: {id: 205, v: 10.0, distance: 20.0}
desired_velocity: 25.0 m/s
lc_left_forbidden: true  // 左侧实线
lc_right_forbidden: true // 右侧有车
```

**输出Snapshot**：
```cpp
original_winner_id: 242
processed_winner_id: 242
action_script[242]: [DDDDD|KKKKK]
// 持续减速，保持车道

final_cost[242]: 22.8  // 代价较高但必须

progress_cost[242]:
  Layer 0 (0-1s): {
    efficiency: {ego: 5.0, leading: 2.0},  // 速度损失大
    safety: {rss: 15.0, occu: 0.0},  // RSS警告
    navigation: {lc: 0.0},
    weight: 1.0,
    total: 22.0  // 安全代价主导
  }
  
  Layer 1 (1-2s): {
    efficiency: {ego: 3.5, leading: 1.5},
    safety: {rss: 5.0, occu: 0.0},  // RSS逐渐满足
    navigation: {lc: 0.0},
    weight: 0.7,
    total: (3.5+1.5+5.0)*0.7 = 7.0
  }
  
  ...

forward_trajs[242]: 
  {t: 0.0, x: 0.0,   y: 0.0, v: 25.0, a: -3.0}
  {t: 0.2, x: 4.85,  y: 0.0, v: 24.4, a: -3.0}
  {t: 0.4, x: 9.61,  y: 0.0, v: 23.8, a: -3.0}
  ...
  {t: 2.0, x: 40.0,  y: 0.0, v: 19.0, a: -3.0}
  ...
  {t: 5.0, x: 90.0,  y: 0.0, v: 10.0, a: 0.0}  // 跟随前车
```

**输出SemanticBehavior**：
```cpp
lat_behavior: LaneKeeping
lon_behavior: Decelerate
actual_desired_velocity: 15.0  // 25 - 10 (减速gap)
ref_lane: Lane 1
```

---

## 4. 代价结构详解

### 4.1 CostStructure 完整分解

```cpp
struct CostStructure {
    // ═══════════════════════════════════════
    // 1. 效率代价 (Efficiency Cost)
    // ═══════════════════════════════════════
    struct EfficiencyCost {
        decimal_t ego_to_desired_vel;
        // 自车与期望速度的差距
        // = unit_cost × |v_ego - v_desired|
        
        decimal_t leading_to_desired_vel;
        // 被前车阻挡的代价
        // = unit_cost × |v_leading - v_desired|
        
        decimal_t ave() const {
            return (ego_to_desired_vel + leading_to_desired_vel) / 2.0;
        }
    } efficiency;
    
    // ═══════════════════════════════════════
    // 2. 安全代价 (Safety Cost)
    // ═══════════════════════════════════════
    struct SafetyCost {
        decimal_t rss;
        // RSS安全距离违反
        // = Σ(max(0, d_safe - d_actual))
        
        decimal_t occu_lane;
        // 违规车道占用
        // = v_ego × unit_cost (如果禁止变道但尝试变道)
        
        decimal_t ave() const {
            return (rss + occu_lane) / 2.0;
        }
    } safety;
    
    // ═══════════════════════════════════════
    // 3. 导航代价 (Navigation Cost)
    // ═══════════════════════════════════════
    struct NavigationCost {
        decimal_t lane_change_preference;
        // 变道偏好代价
        // = v_ego × lc_unit_cost
        // + recommend_reward (如果推荐)
        // + late_penalty (如果延迟)
        
        decimal_t ave() const {
            return lane_change_preference;
        }
    } navigation;
    
    // ═══════════════════════════════════════
    // 4. 时间权重 (折扣因子)
    // ═══════════════════════════════════════
    decimal_t weight;
    // = discount_factor^layer_id
    // 默认 discount_factor = 0.7
    
    // ═══════════════════════════════════════
    // 5. 总代价
    // ═══════════════════════════════════════
    decimal_t ave() const {
        return (efficiency.ave() + 
                safety.ave() + 
                navigation.ave()) * weight;
    }
};
```

### 4.2 代价计算示例

**场景参数**：
```
v_ego = 20.0 m/s
v_desired = 25.0 m/s
v_leading = 15.0 m/s
d_actual = 30.0 m
d_safe_rss = 35.0 m

配置参数:
ego_lack_speed_unit_cost = 0.3
leading_to_desired_unit_cost = 0.2
rss_unit_cost = 1.0
lc_left_unit_cost = 0.015
discount_factor = 0.7
```

**Layer 0 代价**：
```cpp
// Efficiency
efficiency.ego_to_desired_vel = 
    0.3 × |20.0 - 25.0| = 0.3 × 5.0 = 1.5

efficiency.leading_to_desired_vel = 
    0.2 × |15.0 - 25.0| = 0.2 × 10.0 = 2.0

efficiency.ave() = (1.5 + 2.0) / 2.0 = 1.75

// Safety
safety.rss = 
    1.0 × max(0, 35.0 - 30.0) = 1.0 × 5.0 = 5.0

safety.occu_lane = 0.0  // 不违规

safety.ave() = (5.0 + 0.0) / 2.0 = 2.5

// Navigation (如果选择左变道)
navigation.lane_change_preference = 
    20.0 × 0.015 = 0.3

navigation.ave() = 0.3

// Total (Layer 0)
weight = 0.7^0 = 1.0
cost = (1.75 + 2.5 + 0.3) × 1.0 = 4.55
```

**Layer 1 代价**：
```cpp
// 假设已经开始变道，效率提升
efficiency.ave() = 1.2
safety.ave() = 1.0
navigation.ave() = 0.5

weight = 0.7^1 = 0.7
cost = (1.2 + 1.0 + 0.5) × 0.7 = 1.89
```

**总代价**：
```
Total = 4.55 + 1.89 + ... = 12.47
```

---

## 5. 数据流转图

```
┌─────────────────────────────────────────────────────┐
│  Step 1: 准备阶段                                    │
│  ┌─────────────┐    ┌──────────────┐               │
│  │ Semantic Map│───>│ EudmManager  │               │
│  │   Manager   │    │  ::Prepare() │               │
│  └─────────────┘    └──────┬───────┘               │
│                             ↓                        │
│                    ┌────────────────┐               │
│                    │ ego_vehicle    │               │
│                    │ ego_lane_id    │               │
│                    │ lc_info        │               │
│                    └────────┬───────┘               │
└─────────────────────────────┼───────────────────────┘
                              ↓
┌─────────────────────────────┼───────────────────────┐
│  Step 2: 决策阶段           ↓                        │
│                    ┌────────────────┐               │
│                    │ EudmPlanner    │               │
│                    │  ::RunOnce()   │               │
│                    └────────┬───────┘               │
│                             ↓                        │
│              ┌──────────────┴──────────────┐        │
│              ↓                             ↓         │
│     ┌────────────────┐           ┌────────────────┐ │
│     │   DCP Tree     │           │  243 Threads   │ │
│     │ action_script  │───────────>│  Simulation   │ │
│     └────────────────┘           └────────┬───────┘ │
│                                            ↓         │
│                                   ┌────────────────┐ │
│                                   │ sim_res[]      │ │
│                                   │ risky_res[]    │ │
│                                   │ final_cost[]   │ │
│                                   │ forward_trajs[]│ │
│                                   └────────┬───────┘ │
└─────────────────────────────────────────────┼───────┘
                                              ↓
┌─────────────────────────────────────────────┼───────┐
│  Step 3: 评估阶段           ↓                        │
│                    ┌────────────────────┐           │
│                    │ Evaluate & Select  │           │
│                    │   winner_id = 87   │           │
│                    └─────────┬──────────┘           │
│                              ↓                       │
│                    ┌────────────────────┐           │
│                    │  Snapshot          │           │
│                    │  (保存所有结果)     │           │
│                    └─────────┬──────────┘           │
└─────────────────────────────┼────────────────────────┘
                              ↓
┌─────────────────────────────┼────────────────────────┐
│  Step 4: 重选择阶段         ↓                         │
│                    ┌────────────────────┐            │
│                    │ ReselectByContext  │            │
│                    │ processed_id = 87  │            │
│                    └─────────┬──────────┘            │
└─────────────────────────────┼─────────────────────────┘
                              ↓
┌─────────────────────────────┼─────────────────────────┐
│  Step 5: 输出阶段           ↓                          │
│                    ┌────────────────────────┐         │
│                    │  ConstructBehavior     │         │
│                    └─────────┬──────────────┘         │
│                              ↓                         │
│                    ┌────────────────────────┐         │
│                    │  SemanticBehavior      │         │
│                    │  • lat_behavior        │         │
│                    │  • lon_behavior        │         │
│                    │  • desired_velocity    │         │
│                    │  • ref_lane            │         │
│                    │  • forward_trajs       │         │
│                    └─────────┬──────────────┘         │
└─────────────────────────────┼──────────────────────────┘
                              ↓
                     ┌────────────────────┐
                     │   SSC Planner      │
                     │ (轨迹精细化规划)    │
                     └────────────────────┘
```

---

## 6. SSC 时空语义走廊构建细节

本节介绍 SSC 规划器如何基于 EUDM 输出的预测轨迹与环境信息，在 \(s\)-\(d\)-\(t\) 空间中构建**时空语义走廊（Spatio-Temporal Semantic Corridor）**，并给出若干示意图帮助理解。

### 6.1 坐标系与输入数据

SSC 在一条**局部导航参考线**上工作，采用 Frenet 坐标：

```
          d (横向偏移)
          ↑
          │
          │    ● 车辆
          │
          └────────────→ s (沿参考线弧长)
```

- **时间维度 \(t\)** 再加上一维，形成 3D 空间：\((s, d, t)\)。  
- 来自 `SscPlanner::RunOnce()` 的核心输入（已转到 Frenet 坐标系）：  
  - ego 初始 Frenet 状态 `initial_frenet_state_`  
  - EUDM 输出的多条自车预测轨迹：`forward_trajs_fs_`（每条对应一个离散行为）  
  - 周围车辆预测轨迹：`surround_forward_trajs_fs_`  
  - 栅格化静态障碍：`obstacle_grids_fs_`

### 6.2 3D 栅格地图构建（SscMap）

在 `SscPlanner::Init()` 中，根据配置创建 3D 栅格 `SscMap::p_3d_grid_`：

```cpp
// 维度: x=s, y=d, z=t
map_cfg.map_size[0] = cfg_.map_cfg().map_size_x();
map_cfg.map_size[1] = cfg_.map_cfg().map_size_y();
map_cfg.map_size[2] = cfg_.map_cfg().map_size_z();
map_cfg.map_resolution[0] = cfg_.map_cfg().map_resl_x();
map_cfg.map_resolution[1] = cfg_.map_cfg().map_resl_y();
map_cfg.map_resolution[2] = cfg_.map_cfg().map_resl_z();
```

重置与坐标原点设置：

```cpp
// SscMap::ResetSscMap()
start_time_ = ini_fs.time_stamp;
UpdateMapOrigin(ini_fs);

// SscMap::UpdateMapOrigin()
map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;  // s 方向前后范围
map_origin[1] = - (map_size_y-1)*resl_y/2.0;           // d 居中对称
map_origin[2] = ori_fs.time_stamp;                     // t 起点
```

然后用障碍物填充 3D 栅格：

- **静态障碍 FillStaticPart**：  
  - 将 `obstacle_grids_fs` 中每个点 \((s,d)\) 沿时间轴整层“拉伸”，对应所有 \(t\) 网格都标记为占用。  
- **动态障碍 FillDynamicPart**：  
  - 对每辆周围车的 Frenet 轨迹 `FsVehicle`：  
    - 在每个时间戳 \(t_k\)，取车辆 OBB 的顶点集合 \((s,d)\)，映射到当前时间层 `z = t_k` 的 2D 栅格；  
    - 用 `cv::fillPoly` 在该层对车辆占据区域填 100，表示该位置-时间被该车占据。

用一张简化图表示 3D 障碍填充（俯视多个时间切片）：

```
时间切片 t = t0:

   d ↑
     │      #  #      ← 周围车占用区域
     │    #######
     │      ###
     └────────────────→ s

时间切片 t = t1:

   d ↑         ###     ← 车辆向前移动
     │       #######
     │         ###
     └────────────────→ s

堆叠 t0、t1、t2… 即得到 (s,d,t) 三维障碍体
```

### 6.3 沿参考轨迹膨胀时空走廊

核心函数：`SscMap::ConstructCorridorUsingInitialTrajectory()`  
输入为：
- 3D 栅格 `p_3d_grid`（已经填入静态+动态障碍）；  
- 自车某一条预测轨迹 `forward_trajs_fs[i]`（在 Frenet 坐标下的 \(s(t), d(t)\)）。

整体步骤：

1. **用自车轨迹采样一串“种子点” (Seeds)**  

   ```cpp
   // 将 initial_fs 和 forward_trajs_fs[k].frenet_state 映射为 3D 栅格索引
   auto coord_0 = p_grid->GetCoordUsingGlobalPosition({s0, d0, t0});
   auto coord_1 = p_grid->GetCoordUsingGlobalPosition({s1, d1, t1});
   ...
   traj_seeds.push_back(Vec3i(coord_x, coord_y, coord_t));
   ```

   要求：  
   - 在 3D 栅格范围内；  
   - 时间 \(t\) 不早于当前起始时间。

2. **用前两个种子生成“初始立方体” (Initial Cube)**  

   ```cpp
   GetInitialCubeUsingSeed(seed_0, seed_1, &cube);
   // cube.lower_bound / cube.upper_bound 给出 [s,d,t] 的离散包围框
   ```

   - 若立方体内部与障碍体有重叠（`CheckIfCubeIsFree == false`），则该行为对应的走廊无效。

3. **在无障碍的方向上膨胀立方体 (InflateCubeIn3dGrid)**  

   ```cpp
   // 沿 s+/s-/d+/d-/t+ 六个方向膨胀，直到：
   //  - 碰到障碍栅格，或
   //  - 超出动力学边界，或
   //  - 达到配置的最大时间长度
   InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);
   ```

   膨胀的直观几何图：

   ```
   初始种子（2 个点）:

       d ↑
         │      o  (seed_1)
         │
         │  o  (seed_0)
         └────────────→ s

   包络成最小包围框:

       d ↑
         │  +---------+
         │  |   o     |
         │  | o       |
         └──+---------+→ s

   在无障碍方向持续膨胀:

       d ↑
         │  +-----------------+
         │  |        o        |
         │  |   o             |
         │  |                 |
         └──+-----------------+→ s
             ↑      ↑
            s_min  s_max 由障碍分布与动力学约束共同决定
   ```

   同时在时间轴 \(t\) 方向膨胀，形成一个完整的 3D “时空立方体”，代表：
   > 在该时间段内，自车的 \(s,d\) 可在此范围内任意变化而不与障碍碰撞，且满足速度/加速度边界。

4. **沿轨迹依次生成多段立方体（走廊片段）**

   - 遍历后续种子点：  
     - 若当前种子仍在最近一个立方体内，则仅作为该立方体的一个“锚点”；  
     - 若已经跑出当前立方体：
       1. 将当前立方体在时间轴上切断到最后一个仍在内部的种子时间；  
       2. 用“上一个种子”和“当前种子”再生成一个新的初始立方体并重复膨胀；  
       3. 形成下一个时段的走廊立方体。

   得到的走廊是若干个在时间上连续拼接的立方体序列：

   ```
          t ↑
            │       ┌───────┐
            │       │cube 3 │
            │   ┌───┴───────┴───┐
            │   │   cube 2     │
            │┌──┴──────────────┴──┐
            ││      cube 1        │
            └┴────────────────────┴────→ s
                 (d 方向垂直屏幕向外)
   ```

5. **转换为带物理约束的时空语义立方体**

   `SscMap::GetFinalGlobalMetricCubesList()` 将栅格索引转换为物理单位，并为每个维度补充速度/加速度上下界，得到 `SpatioTemporalSemanticCubeNd<2>`：

   ```cpp
   cube.t_lb = z_lb;            // 时间下界
   cube.t_ub = z_ub;            // 时间上界

   cube.p_lb[0] = x_lb;         // s 下界
   cube.p_ub[0] = x_ub;         // s 上界
   cube.v_lb[0] = v_s_min;      // 纵向速度/加速度上下界
   cube.v_ub[0] = v_s_max;
   cube.a_lb[0] = a_s_min;
   cube.a_ub[0] = a_s_max;

   cube.p_lb[1] = y_lb;         // d 下界
   cube.p_ub[1] = y_ub;         // d 上界
   cube.v_lb[1] = -v_d_max;     // 横向速度/加速度对称约束
   cube.v_ub[1] =  v_d_max;
   cube.a_lb[1] = -a_d_max;
   cube.a_ub[1] =  a_d_max;
   ```

   对每条 EUDM 行为（例如“左变道+加速”）都会得到一条对应的时空走廊 `cube_list[i]`。

### 6.4 走廊与轨迹优化的关系

在 `SscPlanner::RunQpOptimization()` 中：

- 对每条行为 `forward_behaviors_[i]`：  
  - 取其对应的 `cube_list[i]`（一串时空语义立方体）；  
  - 将 EUDM 给的参考 Frenet 轨迹离散为一串 \((s(t_k), d(t_k))\) 参考点；  
  - 在走廊约束下求解贝塞尔曲线/五次样条，使之：  
    - 起终点满足指定的 \(s,d,\dot{s},\dot{d}\) 约束；  
    - 整条曲线在任意时间 \(t\) 都落在对应立方体的 \([p_{lb}, p_{ub}]\) 与 \([v_{lb}, v_{ub}], [a_{lb}, a_{ub}]\) 内；  
    - 同时尽量贴近参考点（“靠近 EUDM 预测，但更平滑、更动力学可行”）。

直观理解：

> **EUDM 给的是“粗粒度、语义级”的行为预测；SSC 先在 (s,d,t) 空间用障碍和动力学约束刻出一条“安全通道”（时空走廊），再在这条通道内做连续优化，生成真正要下发给底层控制的精细轨迹。**

---

## 7. 调试技巧

### 7.1 检查决策质量

```bash
# 查看所有候选序列的代价分布
grep "\[Eudm\]\[Result\]" log.txt | head -n 10

# 输出示例:
[Eudm][Result]0 [MMMMM|KKKKK][s:1|r:0|c:15.234]
[Eudm][Result]1 [MMMMM|KKKKL][s:1|r:0|c:16.123]
...
[Eudm][Result]87 [AAMMD|KLKKK][s:1|r:0|c:12.470]  <- 最优
...

# 查看Winner的详细代价
grep "\[Eudm\]\[Result\]\[e;s;n;w:" log.txt
```

### 7.2 分析失败序列

```bash
# 找出所有失败的序列
grep "\[s:0" log.txt

# 示例输出:
[Eudm][Result]128 [DDDDD|RRRRR][s:0|r:0|c:inf] (Collision) 102
[Eudm][Result]215 [AAMMD|LLLLL][s:0|r:0|c:inf] (Pre-deleted)

# 失败原因分类:
# • (Collision): 碰撞
# • (Out of lane): 出界
# • (Pre-deleted): 预剪枝
# • (RSS unsafe): RSS检查失败
```

### 7.3 可视化轨迹

在RViz中：
```
Topic: /eudm/forward_trajs
Type: visualization_msgs/MarkerArray

颜色编码:
• 绿色: 成功且安全的序列
• 黄色: 成功但有风险的序列
• 红色: 失败的序列
• 蓝色加粗: Winner序列
```

---

## 8. 总结

### 8.1 输出数据层次

```
Level 1: 原始仿真结果
  └─> sim_res, risky_res, final_cost (243个)

Level 2: Winner筛选
  └─> original_winner_id, winner_score

Level 3: 上下文调整
  └─> processed_winner_id

Level 4: 语义行为
  └─> SemanticBehavior (供SSC使用)
```

### 8.2 关键数据规模

| 数据项 | 数量 | 大小 |
|-------|------|------|
| 候选序列 | 27 | - |
| 每序列轨迹点 | 25 | 200 bytes |
| 周围车辆数 | 5-10 | 可变 |
| Snapshot总大小 | - | ~5 MB |
| 输出Behavior | 1 | ~5 KB |

### 8.3 性能指标

- **决策频率**: 20 Hz (50ms周期)
- **平均耗时**: 35-45 ms
- **成功率**: >99.5%
- **内存占用**: <50 MB

---

**文档完成** ✅

