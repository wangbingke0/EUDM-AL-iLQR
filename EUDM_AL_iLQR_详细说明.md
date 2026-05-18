# EUDM + AL-iLQR 详细说明

本文聚焦 EPSILON 里最核心的一条主链路：

`SemanticMapManager -> EUDM -> AL-iLQR -> ControlSignal`

这条链路的目标不是单纯“找一条能走的轨迹”，而是先用 EUDM 把复杂交互交通场景压缩成一个离散行为意图，再用 AL-iLQR 在连续状态空间里求一条平滑、可行、带约束的轨迹。

---

## 1. 为什么要把 EUDM 和 AL-iLQR 组合起来

这套组合本质上是在分工：

- EUDM 处理“做什么”
  - 是否保持车道
  - 是否变道
  - 是否加速或减速
  - 在当前场景下哪条离散行为序列最合理

- AL-iLQR 处理“怎么做得更好”
  - 让轨迹满足车辆动力学
  - 让轨迹更平滑
  - 让控制量连续、可执行
  - 让轨迹避开 corridor 约束

所以，EUDM 负责决策，AL-iLQR 负责连续优化。前者偏离散、偏交互，后者偏连续、偏数值优化。

---

## 2. 整体运行链路

在 `planning_integrated` 里，这条链路的组合方式是：

1. `semantic_map_manager` 接收静态地图、动态车辆、车道信息。
2. `EudmPlannerServer` 周期性读取语义地图，运行 EUDM。
3. EUDM 输出：
   - 最终行为 `SemanticBehavior`
   - 前向轨迹 `forward_trajs`
   - 参考车道 `ref_lane`
   - 当前赢家序列 `winner_id`
4. `IlqrPlannerServer` 接收同一份语义地图和 EUDM 输出。
5. iLQR 以 EUDM 的前向轨迹作为参考，构建 Cartesian 优化问题。
6. AL-iLQR 求解后输出连续轨迹，再转换成可执行的 Frenet 兼容结果。
7. 最后通过 `ctrl` 话题发布控制信号，供仿真器或执行器使用。

在代码上，这条组合关系由 `test_ilqr_with_eudm.cc` 串起来：

- EUDM 的 map update 回调把语义地图推给 iLQR。
- iLQR 的行为 update 回调则由 EUDM 提供。

---

## 3. EUDM：行为决策层

### 3.1 EUDM 的定位

EUDM 可以理解为一个带闭环预测的离散行为搜索器。它不是直接输出连续控制，而是输出一个“行为序列”。

它的核心职责是：

- 把周围车辆的交互关系纳入决策
- 预测候选行为序列的可行性
- 对每条候选序列做仿真和打分
- 选出最优序列，再做上下文修正

### 3.2 输入

EUDM 的主要输入来自 `SemanticMapManager`：

- 自车状态
- 周围车辆集合
- 车道网络
- 障碍物信息
- 当前任务参数

其中任务参数里最重要的是：

- 用户期望速度
- 变道限制
- 当前是否处于控制模式

### 3.3 三层结构

EUDM 的实现被分成三层：

#### 1) `EudmPlannerServer`

负责 ROS 接口、消息队列、定时调度。

它会：

- 从 `SemanticMapManager` 接收最新场景
- 在 20Hz 左右触发规划
- 调用 `EudmManager`
- 发布可视化和结果

#### 2) `EudmManager`

负责上下文管理。

它不是单纯“跑一次规划”，而是维护：

- 上一轮赢家序列
- 当前正在执行的动作
- 变道上下文
- HMI/主动变道请求
- 重规划逻辑

这层的价值很大，因为真实车流里，不能每一帧都把行为当成全新问题重新洗牌。

#### 3) `EudmPlanner`

负责真正的行为搜索、前向仿真和代价评估。

### 3.4 DCP 树与候选序列

EUDM 用的是 DCP 树（Decision-making with Closed-loop Prediction）。

它在当前实现里通常是：

- 5 层
- 每层 1 秒
- 总时域 5 秒
- 纵向动作 3 类：保持 / 加速 / 减速
- 横向动作 3 类：保持车道 / 左变道 / 右变道

因此理论候选序列为：

`3 x 9 = 27`

这里有一个关键约束：纵向动作在整个 5 秒内保持一致，也就是不会出现“前两秒加速、后两秒减速”这种混合纵向模式。

### 3.5 评分与仿真

EUDM 对每条候选序列做多线程前向仿真，然后计算：

- 是否成功
- 是否有风险
- 总代价
- 分层代价
- 尾部代价
- 自车轨迹
- 周车轨迹

代价结构大致分三块：

- efficiency：效率
- safety：安全
- navigation：导航/变道偏好

这三块不是由求解器“自动学出来”的，而是代码里直接写好的启发式代价。对一条候选序列，EUDM 最终做的是：

```math
J_{\text{seq}}=\sum_{i=1}^{L} J_i + J_{\text{tail}}
```

其中每一层的代价为：

```math
J_i=t_i\cdot(\text{Eff}_i+\text{Safe}_i+\text{Nav}_i)
```

而 `CostStructure::ave()` 里对应的是：

```math
\text{Eff}_i=\frac{J^{ego}_{eff}+J^{lead}_{eff}}{2},\quad
\text{Safe}_i=\frac{J^{rss}+J^{occu}}{2},\quad
\text{Nav}_i=J^{nav}
```

#### efficiency

自车速度偏离目标速度时的效率损失：

```math
J^{ego}_{eff}=
\begin{cases}
w_{lack}\cdot |v_e-v_{des}|, & v_e < v_{des} \\
w_{over}\cdot |v_e-v_{des}-\Delta v|, & v_e > v_{des}+\Delta v \\
0, & \text{otherwise}
\end{cases}
```

如果被前车阻挡，还会额外加一项：

```math
J^{lead}_{eff}=
\max(\rho_{min},\rho_{dist})
\Big(
w_{ego}\cdot \max(v_e-v_l,0)
+w_{lead}\cdot \max(v_{des}-v_l,0)
\Big)
```

其中 \(\rho_{dist}\) 是距离残差比例，\(\rho_{min}\) 是最小比例下限。

#### safety

RSS 风险项是按碰撞危险程度加罚的：

```math
J^{rss}\mathrel{+}=
\begin{cases}
c_{over}\,v_e\,10^{p_{over}|v_e-v^{up}_{rss}|}, & \text{TooFast} \\
c_{lack}\,v_e\,10^{p_{lack}|v_e-v^{low}_{rss}|}, & \text{TooSlow} \\
0, & \text{safe}
\end{cases}
```

如果动作违反车道占用/变道限制，还会有占用惩罚：

```math
J^{occu}=
\begin{cases}
v_e\cdot c_{occu}, & \text{禁止左变却选左变} \\
v_e\cdot c_{occu}, & \text{禁止右变却选右变} \\
0, & \text{otherwise}
\end{cases}
```

要注意，严格碰撞检查是硬过滤，不是代价项。只要碰撞，序列直接失败。

#### navigation

变道意图的基础惩罚：

```math
J^{nav}=\max(v_e,v_{lb})\cdot c_{lanechange}
```

取消变道时：

```math
J^{nav}=\max(v_e,v_{lb})\cdot c_{cancel}
```

如果当前变道方向是推荐方向，还会给奖励：

```math
J^{nav}=-\max(v_e,v_{lb})\cdot r_{recommend}
```

如果“该变但这一层还没变”，则会附加迟滞惩罚：

```math
J^{nav}\mathrel{+}=\max(v_e,v_{lb})\cdot c_{late}
```

最后，EUDM 选的是

```math
\arg\min_i J_{\text{seq}}(i)
```

也就是把所有成功序列的总分算完后，直接取最小值。

这也是它能处理“不是只有一条正确答案”的原因。EUDM 不只是筛掉碰撞序列，而是会比较不同动作在效率、安全和导航偏好上的权衡。

### 3.6 原始赢家与上下文重选

EUDM 有两个赢家概念：

- `original_winner_id`
- `processed_winner_id`

前者是纯代价最优序列，后者是结合上下文后的最终输出。

为什么要再选一次？

因为真实车流里，行为不能只看当下最小代价，还要看：

- 当前变道请求是否已经稳定
- 当前变道是否应该立即执行
- 是否处于保持阶段
- 是否存在取消变道的上下文

这就是 `EudmManager::ReselectByContext()` 的作用。它会把上一轮动作序列、当前时间、变道上下文一起考虑，避免出现“数值上最优，但行为上很跳”的结果。

### 3.7 输出

EUDM 最终输出的是 `SemanticBehavior`，核心字段包括：

- `lat_behavior`
- `lon_behavior`
- `actual_desired_velocity`
- `forward_trajs`
- `ref_lane`
- `forward_behaviors`
- `surround_trajs`

这份输出会直接喂给后端轨迹层，尤其是 iLQR。

---

## 4. AL-iLQR：连续轨迹优化层

### 4.1 它在做什么

AL-iLQR 不是单纯“修一条线”，而是在一个带动力学和约束的连续优化问题上找最优控制序列。

它使用的是 `altro-cpp` 的增广拉格朗日 iLQR 求解器。

### 4.2 状态与控制

当前实现里：

- 状态 `x = [x, y, yaw, v, a, kappa]`
- 控制 `u = [jerk, dkappa]`

这意味着它不仅跟踪位置和速度，还显式建模：

- 加速度
- 曲率
- 曲率变化率

这比传统只看 `x, y, yaw` 的规划更细。

### 4.3 动力学

连续动力学大致是：

- `dx/dt = v * cos(yaw)`
- `dy/dt = v * sin(yaw)`
- `dyaw/dt = v * kappa`
- `dv/dt = a`
- `da/dt = jerk`
- `dkappa/dt = dkappa`

这套模型的好处是：

- 轨迹更贴近真实车辆
- 控制量更连续
- 约束更容易落到优化问题里

### 4.4 参考轨迹来自哪里

iLQR 的参考轨迹并不是凭空生成的，而是来自 EUDM 的前向轨迹。

流程是：

1. 取 EUDM 的 forward trajectory
2. 转成 Frenet / Cartesian 表达
3. 按固定时间步长采样
4. 形成 `reference_cartesian_states_`
5. 作为 iLQR 的跟踪目标

当前默认配置里通常是：

- `N = 50`
- `dt = 0.1s`
- 总时域约 5 秒

### 4.5 为什么要 warm start

直接把 AL-iLQR 扔进复杂场景，收敛可能会很差。

所以这里先做了 LQR warm start：

- 用线性二次近似先跑一版初值
- 给增广拉格朗日求解器一个更好的起点

如果 warm start 失败，还会退化成零控制或参考轨迹兜底。

### 4.6 代价函数

阶段代价主要是：

- 位置跟踪
- 航向跟踪
- 速度跟踪
- 加速度正则
- 曲率正则
- jerk 正则
- `dkappa` 正则

终端代价会更重，强调终点状态收敛。

简单说：

- 前半段要“跟得上参考线”
- 后半段要“收得住”

### 4.7 约束

AL-iLQR 里主要有三类约束：

#### 1) 控制约束

- `jerk` 上下界
- `dkappa` 上下界

#### 2) 状态约束

- 速度上下界
- 加速度上下界
- 曲率上下界

#### 3) corridor 约束

这是最关键的一层。

它沿用了 SSC 风格的时空走廊表达，把可行区域变成一组随时间变化的几何约束。这样 iLQR 不是在“空旷平面”里找轨迹，而是在受限通道里找轨迹。

### 4.8 求解流程

iLQR 的核心求解器在 `IlqrAlIlqrSolver::Solve()` 中完成，流程大致是：

1. 构建问题
2. 设置初始状态
3. 设置离散动力学
4. 设置阶段代价和终端代价
5. 添加控制边界
6. 添加状态边界
7. 添加 corridor 约束
8. 做 LQR warm start
9. 用 AL-iLQR 求解
10. 校验结果，必要时回退

### 4.9 结果校验与回退

求解完后，代码不是立刻接受结果，而是会检查：

- 状态是否有限
- 状态边界是否超限
- 相邻段位移是否过大
- 对参考轨迹的偏离是否过大

如果优化结果质量不够，就回退到 warm start 轨迹。

这点很实用，因为它让系统“宁愿保守，也不要输出一条数值上解出来、但看起来很怪的轨迹”。

### 4.10 输出

AL-iLQR 最终输出：

- `CartesianTrajectory`
- `IlqrFrenetTrajectory`
- 可视化轨迹
- `ctrl` 控制信号

也就是说，它最后还是会包装成规划系统能继续使用的轨迹接口，而不是只给一个优化器内部结果。

---

## 5. EUDM 和 AL-iLQR 的接口关系

这两者的接口关系很清晰：

### EUDM -> iLQR

EUDM 给 iLQR：

- 行为意图
- 参考车道
- 前向参考轨迹
- 周围车辆预测
- 当前场景语义

### iLQR -> 执行层

iLQR 输出：

- 平滑轨迹
- 控制信号
- 可视化结果

### 中间的关键点

EUDM 负责离散决策，iLQR 负责连续优化，中间通过参考轨迹和 corridor 连接。

所以你可以把它看成：

`离散意图 -> 连续可执行轨迹`

这是这套系统最核心的思想。

---

## 6. 运行入口

最直接的入口是：

- `planning_integrated/launch/test_ilqr_with_eudm_ros.launch`
- `planning_integrated/src/test_ilqr_with_eudm.cc`

这里会：

- 创建 `SemanticMapManager`
- 启动 `EudmPlannerServer`
- 启动 `IlqrPlannerServer`
- 把 EUDM 的输出推给 iLQR

如果你只是想看 EUDM + SSC 的老链路，也可以对照：

- `test_ssc_with_eudm_ros.launch`

但这篇文档建议优先看 EUDM + AL-iLQR 这条新链路。

---

## 7. 调参重点

### EUDM 侧

重点看：

- 决策时域
- 每层时长
- 代价权重
- RSS 配置
- 变道上下文参数
- HMI/主动变道触发条件

### iLQR 侧

重点看：

- `time_horizon`
- `dt`
- `num_knot_points`
- `weight_x/y/yaw/v/a/kappa`
- `weight_jerk/dkappa`
- `weight_terminal_*`
- `max_velocity`
- `max_acceleration`
- `max_curvature`
- `max_dkappa`
- `max_outer_iterations`
- `max_inner_iterations`
- `constraint_tolerance`
- `cost_tolerance`

通常调参顺序建议是：

1. 先保证参考轨迹合理
2. 再调代价权重
3. 再调约束边界
4. 最后调求解器迭代和容差

---

## 8. 常见问题怎么看

### 1) EUDM 没有输出 winner

优先看：

- 周围车辆输入是否完整
- RSS/碰撞检查是否过严
- 候选序列是否全部失败
- 参考速度是否过激

### 2) EUDM 有输出，但 iLQR 不收敛

优先看：

- EUDM 参考轨迹是否跳变太大
- 初始速度是否太低
- corridor 是否过窄
- 曲率和 `dkappa` 限制是否过紧
- warm start 是否失败

### 3) iLQR 输出轨迹很怪

优先看：

- reference sampling 是否异常
- 初始状态是否和参考轨迹首点一致
- 是否触发了回退
- 是否出现了数值越界

### 4) 轨迹好看但控制不稳定

优先看：

- `use_sim_state`
- 发布的 `ctrl` 时间戳
- 轨迹与执行状态是否同步

---

## 9. 一句话总结

EUDM 解决的是“在复杂交互里选哪种行为”，AL-iLQR 解决的是“把这个行为变成一条平滑、可行、可控的连续轨迹”。

前者给意图，后者给落地。两个模块一前一后，构成了 EPSILON 里最有代表性的决策规划主线。

---

## 10. 对话补充要点

这一节把前面对话里最容易被问到、也最容易混淆的几个实现点单独整理出来。

### 10.1 EUDM 不是“先筛碰撞，再选一个”

EUDM 的做法不是只做碰撞过滤，而是对每条候选动作序列做完整前向仿真，然后比较不同序列在效率、安全、导航偏好上的总分，最后取最小代价序列。也就是说，它不是一个硬规则筛子，而是一个“仿真 + 评分 + 排名”的离散行为搜索器。

### 10.2 27 条候选动作序列怎么来的

`DcpTree` 会枚举 3 类纵向动作：

- `Maintain`
- `Accelerate`
- `Decelerate`

对于每一种纵向动作，再结合 1 个“当前保持车道”的起始状态，枚举一次变道发生的时机与方向。默认 5 层规划里，每个纵向动作会形成 9 条序列：

- 全程保持车道 1 条
- 在第 1 到第 4 层中的某一层发起左变道或右变道，共 8 条

因此总数是：

```math
3 \times 9 = 27
```

这也解释了“为什么不是允许多次变道”：因为这里的 DCP 树本身就是按“单次变道意图”来展开的，序列结构里不会把多次左右来回切换当作常规候选。

### 10.3 前向仿真里静态障碍物和动态障碍物有没有区分

有，但区分发生在上游，不在 AL-iLQR 内部。

- 静态障碍物先进入 `obstacle_set`，再被转成 `obstacle_grids`
- 动态障碍物先进入 `vehicle_set`，再被转成周车轨迹 `surround_forward_trajs`
- `SSC` 构建走廊时，明确分成 `FillStaticPart()` 和 `FillDynamicPart()`
- `AL-iLQR` 拿到的是已经融合后的时空走廊，不再关心某个约束来自静态还是动态障碍物

所以更准确地说，**静态/动态的区分存在于语义建图和 SSC 走廊构造阶段，优化器本身只看可行走廊**。

### 10.4 前向仿真的核心细节

EUDM 对每条候选序列会按层推进，每层里又拆成多个更小时间步，逐步更新：

- 自车状态
- 周车状态
- 当前层动作
- 当前层的风险和代价

其中：

- 纵向动作会影响 IDM 参数，尤其是 `kDesiredVelocity`
- 横向动作会决定当前层是否保持车道、左变道或右变道
- 每一层结束后会更新后续层的动作序列和上下文

这就是它“闭环预测”的含义。

### 10.5 IDM 参数是怎么设的

代码里不是固定一个 IDM，而是会根据纵向动作调整参数。

默认配置里，EUDM 的 IDM 基础参数来自 `eudm_config.pb.txt`，常见值包括：

- 自车 `min_spacing = 2.0`, `head_time = 1.0`
- 周车 `min_spacing = 1.0`, `head_time = 0.5`
- `acc_cmd_vel_gap = 10.0`
- `dec_cmd_vel_gap = 10.0`
- `lon_aggressive_ratio = 0.25`

纵向动作对期望速度的影响大致是：

- `Accelerate`
  - `kDesiredVelocity = min(当前速度 + 10, 用户期望速度)`
  - 同时把 `min_spacing` 和 `head_time` 乘以 `1 - lon_aggressive_ratio`
  - 结果更激进，更愿意贴近目标速度
- `Decelerate`
  - `kDesiredVelocity = min(max(当前速度 - 10, 0), 用户期望速度)`
  - 结果更保守
- `Maintain`
  - `kDesiredVelocity = min(当前期望速度, 用户期望速度)`

对周车而言，代码里也会根据其当前状态和加减速意图设置 `kDesiredVelocity`，所以前向仿真不是“所有车都用一个固定 IDM”，而是会随行为动作动态调整。

### 10.6 效率代价里为什么要改写原论文公式

这个项目里的实现比论文写法更工程化一些。更合理的解释是：

- 自车速度低于期望速度时才惩罚效率损失
- 前车速度低于期望速度时，才把“被前车阻塞”作为效率损失
- 如果前车本来就快于期望速度，不应该单独惩罚它

因此，文档里更建议把效率代价写成“自车速度偏差 + 前车阻塞项”的形式，而不是直接把 `|v_lead - v_des|` 也写成效率损失。

### 10.7 AL-iLQR 里到底怎么实现求解

AL-iLQR 的实现顺序可以概括成：

1. 构建 `Problem(N)`
2. 填入连续车辆模型的离散化版本
3. 建立二次型跟踪代价
4. 加入控制约束、状态约束和 corridor 约束
5. 用 LQR warm start 生成初值
6. 调用 `AugmentedLagrangianiLQR`
7. 做外层乘子更新和 penalty 更新
8. 检查求解质量，不通过就回退

它本质上是“增广拉格朗日外循环 + iLQR 内循环”，而不是单次牛顿优化。
