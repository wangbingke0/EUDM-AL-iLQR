# LQR 热启动轨迹异常分析

## 问题现象
从图片可以看到，黄色轨迹（LQR热启动轨迹）形成了一个"U"形或"J"形的循环路径，这明显不正常。

## 可能原因分析

### 1. **反馈增益过大导致过度修正**

**问题位置**：`ilqr_planner.cpp:568-573`

```cpp
// Compute state error
altro::VectorXd dx = x - x_ref[k];

// Compute control: u = u_ref + K * dx
altro::VectorXd u = u_ref[k] + K[k] * dx;
```

**原因**：
- 从配置来看，`weight_y = 10.0` 和 `weight_terminal_y = 100.0` 都很大
- 这会导致LQR反馈增益矩阵K中的横向位置（y）分量很大
- 如果初始状态与参考轨迹有横向偏差，`dx`会很大
- `u = K * dx` 会产生很大的控制输入，即使被裁剪到边界内，也可能导致轨迹剧烈变化

### 2. **非线性动力学传播不稳定**

**问题位置**：`ilqr_planner.cpp:583-586`

```cpp
// Forward propagate state using nonlinear dynamics
altro::VectorXd xdot(kStateDim);
dynamics.Evaluate(x, u, k * dt, xdot);
x = x + dt * xdot;
```

**原因**：
- 当控制输入u很大时，特别是曲率变化率（dkappa）很大
- 非线性动力学 `dyaw/dt = v * kappa` 会导致航向角快速变化
- 如果速度v很小但曲率kappa很大，或者曲率变化率dkappa很大，会导致车辆快速转向
- 这可能导致轨迹形成循环

### 3. **参考轨迹与初始状态不匹配**

**问题位置**：`ilqr_planner.cpp:501-523`

```cpp
// Build reference trajectory
for (int k = 0; k <= N; ++k) {
    if (k < static_cast<int>(reference_cartesian_states_.size())) {
        const auto& ref = reference_cartesian_states_[k];
        x_ref[k] = ...;
    } else if (!reference_cartesian_states_.empty()) {
        const auto& ref = reference_cartesian_states_.back();
        x_ref[k] = ...;  // 使用最后一个状态
    } else {
        x_ref[k] = x0;   // 使用初始状态
    }
}
```

**原因**：
- 如果参考轨迹太短（小于N），后面的点会重复使用最后一个状态
- 如果参考轨迹为空，所有点都使用初始状态x0
- 这会导致参考轨迹与当前车辆状态不匹配
- LQR会试图修正这个不匹配，但可能过度修正

### 4. **控制输入裁剪后的累积效应**

**问题位置**：`ilqr_planner.cpp:575-577`

```cpp
// Clip control to bounds
u(0) = std::max(-cfg_.max_jerk, std::min(cfg_.max_jerk, u(0)));
u(1) = std::max(-cfg_.max_dkappa, std::min(cfg_.max_dkappa, u(1)));
```

**原因**：
- 如果计算出的控制输入u超出边界，会被裁剪
- 但裁剪后的控制输入可能仍然很大
- 每一步的修正累积起来，可能导致轨迹偏离参考轨迹越来越远
- 形成循环或振荡

### 5. **速度-曲率耦合问题**

**问题位置**：`ilqr_planner.cpp:588-589`

```cpp
// Ensure velocity is within bounds
x(3) = std::max(cfg_.min_velocity, std::min(cfg_.max_velocity, x(3)));
```

**原因**：
- 如果速度很小（接近min_velocity = 0.1 m/s），但曲率kappa很大
- 根据 `dyaw/dt = v * kappa`，航向角变化率仍然可能很大
- 这会导致车辆在原地或低速时快速转向，形成循环

### 6. **Riccati方程求解可能不稳定**

**问题位置**：`ilqr_planner.cpp:551-561`

```cpp
// Compute feedback gain: K = -(R + B'PB)^(-1) * B'PA
altro::MatrixXd R_BPB = R + B[k].transpose() * P * B[k];
altro::MatrixXd K_k = -R_BPB.inverse() * B[k].transpose() * P * A[k];
```

**原因**：
- 如果 `R_BPB` 矩阵接近奇异（条件数很大），求逆可能不稳定
- 这会导致反馈增益K非常大或包含NaN/Inf
- 即使有安全检查，也可能导致轨迹异常

## 解决方案建议

### 1. **降低反馈增益权重**
- 减小 `weight_y` 和 `weight_terminal_y`
- 或者对反馈增益K进行缩放：`K_scaled = alpha * K`，其中 `alpha < 1.0`

### 2. **限制状态误差**
- 对状态误差dx进行裁剪：`dx_clamped = clamp(dx, -dx_max, dx_max)`
- 防止过大的状态误差导致过大的控制输入

### 3. **改进参考轨迹**
- 确保参考轨迹长度足够（至少50米，已实现）
- 确保参考轨迹与初始状态连续

### 4. **添加稳定性检查**
- 检查反馈增益K的条件数
- 如果条件数太大，使用更保守的增益

### 5. **限制控制输入的累积效应**
- 使用更小的控制权重 `weight_jerk` 和 `weight_dkappa`
- 或者对控制输入进行平滑处理

### 6. **检查速度-曲率耦合**
- 在低速度时限制曲率：`if (v < 0.5) kappa = 0`
- 这已经在代码中部分实现了，但可能需要更严格

