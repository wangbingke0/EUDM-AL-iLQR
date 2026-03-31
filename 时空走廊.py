import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np


def draw_cube(ax, x_min, x_max, y_min, y_max, t_min, t_max, color='lightblue', alpha=0.3):
    """绘制3D立方体表示时空走廊"""
    corners = np.array([
        [x_min, y_min, t_min],
        [x_max, y_min, t_min],
        [x_max, y_max, t_min],
        [x_min, y_max, t_min],
        [x_min, y_min, t_max],
        [x_max, y_min, t_max],
        [x_max, y_max, t_max],
        [x_min, y_max, t_max],
    ])
    faces = [
        [corners[i] for i in [0, 1, 2, 3]],
        [corners[i] for i in [4, 5, 6, 7]],
        [corners[i] for i in [0, 1, 5, 4]],
        [corners[i] for i in [2, 3, 7, 6]],
        [corners[i] for i in [1, 2, 6, 5]],
        [corners[i] for i in [4, 7, 3, 0]],
    ]
    poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='k', linewidths=0.5)
    ax.add_collection3d(poly)


def draw_wire_cube(ax, x_min, x_max, y_min, y_max, t_min, t_max, color='limegreen', lw=1.0, alpha=0.9):
    """绘制立方体线框，便于突出边界结构"""
    corners = np.array([
        [x_min, y_min, t_min],
        [x_max, y_min, t_min],
        [x_max, y_max, t_min],
        [x_min, y_max, t_min],
        [x_min, y_min, t_max],
        [x_max, y_min, t_max],
        [x_max, y_max, t_max],
        [x_min, y_max, t_max],
    ])
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    for i, j in edges:
        ax.plot(
            [corners[i, 0], corners[j, 0]],
            [corners[i, 1], corners[j, 1]],
            [corners[i, 2], corners[j, 2]],
            color=color,
            linewidth=lw,
            alpha=alpha,
        )


def draw_lane_boundary_surfaces(ax, s_min, s_max, t_max, d_min, d_max):
    """绘制车道边界面（蓝色）"""
    s_grid = np.linspace(s_min, s_max, 40)
    t_grid = np.linspace(0.0, t_max, 40)
    s_mesh, t_mesh = np.meshgrid(s_grid, t_grid)

    d_low = np.full_like(s_mesh, d_min)
    d_high = np.full_like(s_mesh, d_max)
    ax.plot_surface(s_mesh, d_low, t_mesh, color='dodgerblue', alpha=0.08, linewidth=0)
    ax.plot_surface(s_mesh, d_high, t_mesh, color='dodgerblue', alpha=0.08, linewidth=0)

    ax.plot([s_min, s_max], [d_min, d_min], [0, 0], color='dodgerblue', linestyle=':', linewidth=2.0, alpha=0.9)
    ax.plot([s_min, s_max], [d_max, d_max], [0, 0], color='dodgerblue', linestyle=':', linewidth=2.0, alpha=0.9)
    ax.plot([s_max, s_max], [d_min, d_min], [0, t_max], color='dodgerblue', linestyle=':', linewidth=1.8, alpha=0.9)
    ax.plot([s_max, s_max], [d_max, d_max], [0, t_max], color='dodgerblue', linestyle=':', linewidth=1.8, alpha=0.9)


def draw_speed_limit_surfaces(ax, d_min, d_max, t_max, s0, v_min, v_max):
    """绘制速度限制对应的 s-t 斜边界面（灰色）"""
    t_grid = np.linspace(0.0, t_max, 45)
    d_grid = np.linspace(d_min, d_max, 2)
    t_mesh, d_mesh = np.meshgrid(t_grid, d_grid)

    s_low = s0 + v_min * t_mesh
    s_high = s0 + v_max * t_mesh
    ax.plot_surface(s_low, d_mesh, t_mesh, color='lightcoral', alpha=0.08, linewidth=0)
    ax.plot_surface(s_high, d_mesh, t_mesh, color='lightcoral', alpha=0.08, linewidth=0)

    ax.plot(s0 + v_min * t_grid, np.full_like(t_grid, d_min), t_grid, color='lightcoral', linestyle=':', linewidth=2.0)
    ax.plot(s0 + v_min * t_grid, np.full_like(t_grid, d_max), t_grid, color='lightcoral', linestyle=':', linewidth=2.0)
    ax.plot(s0 + v_max * t_grid, np.full_like(t_grid, d_min), t_grid, color='lightcoral', linestyle=':', linewidth=2.0)
    ax.plot(s0 + v_max * t_grid, np.full_like(t_grid, d_max), t_grid, color='lightcoral', linestyle=':', linewidth=2.0)


def clip_interval_by_obstacle(s_min, s_max, s_obs_min, s_obs_max, s_ref):
    """
    在 s 轴上从可行区间中切掉障碍区间。
    规则：优先保留包含参考点 s_ref 一侧的可行段。
    """
    if s_max <= s_obs_min or s_min >= s_obs_max:
        return s_min, s_max

    if s_ref <= s_obs_min:
        s_max = min(s_max, s_obs_min)
    elif s_ref >= s_obs_max:
        s_min = max(s_min, s_obs_max)
    else:
        left_width = max(0.0, s_obs_min - s_min)
        right_width = max(0.0, s_max - s_obs_max)
        if right_width > left_width:
            s_min = max(s_min, s_obs_max)
        else:
            s_max = min(s_max, s_obs_min)
    return s_min, s_max


def draw_vehicle_obstacle(ax, s_center, d_center, t_start, t_end, length=4.5, width=1.8, color='red', alpha=0.6):
    """绘制车辆障碍物的时空占用区域"""
    s_min = s_center - length/2
    s_max = s_center + length/2
    d_min = d_center - width/2
    d_max = d_center + width/2
    draw_cube(ax, s_min, s_max, d_min, d_max, t_start, t_end, color=color, alpha=alpha)


def draw_dynamic_vehicle_tube(ax, t_edges, s_ref, d_ref, t_ref, length=4.5, width=1.8, color='red', alpha=0.28):
    """按时间切片绘制动态车辆占用（时空管）"""
    for i in range(len(t_edges) - 1):
        t0, t1 = t_edges[i], t_edges[i + 1]
        tm = 0.5 * (t0 + t1)
        s_center = np.interp(tm, t_ref, s_ref)
        d_center = np.interp(tm, t_ref, d_ref)
        draw_vehicle_obstacle(
            ax, s_center, d_center, t0, t1,
            length=length, width=width, color=color, alpha=alpha
        )


def polynomial_trajectory(t, coeffs_s, coeffs_d):
    """使用多项式生成平滑轨迹
    coeffs_s: s方向的多项式系数 [a0, a1, a2, a3, a4, a5] 对应 a0 + a1*t + a2*t^2 + ... + a5*t^5
    coeffs_d: d方向的多项式系数
    """
    s = sum(coeffs_s[i] * (t ** i) for i in range(len(coeffs_s)))
    d = sum(coeffs_d[i] * (t ** i) for i in range(len(coeffs_d)))
    return s, d


def generate_lane_change_polynomial(t_total=5.0):
    """生成左变道的多项式轨迹参数"""
    # 边界条件设定
    # t=0: s=0, s'=15, s''=0 (初始位置、速度、加速度)
    # t=5: s=75, s'=15, s''=0 (终点位置、速度、加速度)
    
    # s方向：匀速运动 s = s0 + v*t
    coeffs_s = [0.0, 15.0, 0.0, 0.0, 0.0, 0.0]  # s = 15*t
    
    # d方向：变道轨迹，使用5次多项式满足边界条件
    # t=0: d=0, d'=0, d''=0
    # t=5: d=3.5, d'=0, d''=0
    # 变道主要在t=1.5到t=3.5之间进行
    
    # 使用分段多项式：0-1.5s保持直行，1.5-3.5s变道，3.5-5s保持直行
    return coeffs_s


def smooth_lane_change_trajectory(t_array):
    """生成平滑的变道轨迹"""
    s_traj = 15.0 * t_array  # 匀速15m/s
    
    # 使用sigmoid函数的变形来生成平滑的变道轨迹
    d_traj = np.zeros_like(t_array)
    for i, t in enumerate(t_array):
        if t <= 1.5:
            d_traj[i] = 0.0
        elif t >= 3.5:
            d_traj[i] = 3.5
        else:
            # 在1.5-3.5秒之间使用5次多项式进行平滑变道
            tau = (t - 1.5) / (3.5 - 1.5)  # 归一化时间 [0,1]
            # 5次多项式：满足边界条件 f(0)=0, f'(0)=0, f''(0)=0, f(1)=1, f'(1)=0, f''(1)=0
            d_traj[i] = 3.5 * (10*tau**3 - 15*tau**4 + 6*tau**5)
    
    return s_traj, d_traj


def generate_spatiotemporal_corridor(
    t_ref,
    ego_s_ref,
    ego_d_ref,
    t_edges,
    lane_width=3.5,
    t_start=0.0,
    v_min=8.0,
    v_max=20.0,
    static_obstacles=None,
    dynamic_obstacles=None,
):
    """
    根据参考轨迹、车道边界、速度边界、静态/动态障碍，
    自动生成一组 s-d-t corridor 立方体。
    逻辑：
    1) 围绕自车参考点生成基础包络；
    2) 叠加速度边界与车道边界；
    3) 叠加静态/动态障碍占用并在 s 轴裁剪；
    4) 保证每段走廊最小厚度，避免退化。
    """
    if static_obstacles is None:
        static_obstacles = []
    if dynamic_obstacles is None:
        dynamic_obstacles = []

    road_d_min = -lane_width / 2.0
    road_d_max = lane_width * 1.5  # 两车道：[-1.75, 5.25]
    s_origin = float(np.interp(t_start, t_ref, ego_s_ref))

    # 自车走廊基础包络
    corridor_front = 20.0
    corridor_back = 6.0
    corridor_half_d = 1.4
    min_s_width = 4.0

    # 安全边际
    safe_front = 6.0
    safe_back = 3.0
    safe_lat = 0.35
    static_safe_front = 2.0
    static_safe_back = 1.5
    static_safe_lat = 0.15

    ego_half_width = 0.9
    edge_margin = 0.2

    cubes = []
    for i in range(len(t_edges) - 1):
        t0, t1 = t_edges[i], t_edges[i + 1]
        tm = 0.5 * (t0 + t1)

        s_ego = np.interp(tm, t_ref, ego_s_ref)
        d_ego = np.interp(tm, t_ref, ego_d_ref)

        # 1) 先给一个围绕参考点的基础 corridor
        # 2) 叠加速度边界：s0 + v_min * t <= s <= s0 + v_max * t
        s_speed_min = s_origin + v_min * (t0 - t_start)
        s_speed_max = s_origin + v_max * (t1 - t_start)

        s_min = max(0.0, s_ego - corridor_back, s_speed_min)
        s_max = min(s_ego + corridor_front, s_speed_max)
        d_min = max(road_d_min + ego_half_width + edge_margin, d_ego - corridor_half_d)
        d_max = min(road_d_max - ego_half_width - edge_margin, d_ego + corridor_half_d)

        # 3) 静态障碍裁剪（静态障碍在整个时域都存在）
        for obs in static_obstacles:
            obs_s_min, obs_s_max, obs_d_min, obs_d_max = obs
            obs_d_min = obs_d_min - static_safe_lat
            obs_d_max = obs_d_max + static_safe_lat
            lateral_overlap = not (d_max <= obs_d_min or d_min >= obs_d_max)
            if not lateral_overlap:
                continue
            s_obs_min = obs_s_min - static_safe_back
            s_obs_max = obs_s_max + static_safe_front
            s_min, s_max = clip_interval_by_obstacle(s_min, s_max, s_obs_min, s_obs_max, s_ego)

        # 4) 动态障碍裁剪
        for obs in dynamic_obstacles:
            obs_s = np.interp(tm, obs["t_ref"], obs["s_ref"])
            obs_d = np.interp(tm, obs["t_ref"], obs["d_ref"])
            obs_s_min = obs_s - obs["length"] / 2.0 - obs.get("safe_back", safe_back)
            obs_s_max = obs_s + obs["length"] / 2.0 + obs.get("safe_front", safe_front)
            obs_d_min = obs_d - obs["width"] / 2.0 - obs.get("safe_lat", safe_lat)
            obs_d_max = obs_d + obs["width"] / 2.0 + obs.get("safe_lat", safe_lat)
            lateral_overlap = not (d_max <= obs_d_min or d_min >= obs_d_max)
            if lateral_overlap:
                s_min, s_max = clip_interval_by_obstacle(s_min, s_max, obs_s_min, obs_s_max, s_ego)

        # 保证横向不退化
        if d_max - d_min < 0.8:
            d_center = np.clip(d_ego, road_d_min + ego_half_width + edge_margin, road_d_max - ego_half_width - edge_margin)
            d_min = d_center - 0.4
            d_max = d_center + 0.4

        # 5) 防退化：保证 corridor 有最小纵向厚度
        if s_max - s_min < min_s_width:
            center = float(np.clip(s_ego, s_speed_min, s_speed_max))
            s_min = max(0.0, center - min_s_width / 2.0, s_speed_min)
            s_max = min(s_min + min_s_width, s_speed_max)
            if s_max - s_min < min_s_width:
                s_min = max(0.0, s_speed_max - min_s_width)
                s_max = s_speed_max

        cubes.append((s_min, s_max, d_min, d_max, t0, t1))

    return cubes

def plot_3d_corridor_lane_change():
    """绘制 SSC 论文风格的 3D 时空走廊示意图"""
    fig = plt.figure(figsize=(24, 12), dpi=140)
    fig.patch.set_facecolor('#f2f2f2')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#f2f2f2')
    # 尽量铺满画布，减少留白
    ax.set_position([0.00, 0.00, 1.00, 0.95])

    # 基本场景参数
    lane_width = 3.5
    lane_d_min = -lane_width / 2.0
    lane_d_max = lane_width * 1.5
    t_end = 6.0
    s_view_max = 105.0
    v_min = 8.0
    v_max = 20.0

    # 时间轴与轨迹
    t_sim = np.linspace(0.0, t_end, 720)
    ego_s_traj, ego_d_traj = smooth_lane_change_trajectory(t_sim)

    # 动态障碍（两辆车）
    dyn1_s = 40.0 + 12.0 * t_sim
    dyn1_d = np.zeros_like(t_sim)
    dyn2_s = 18.0 + 16.0 * t_sim
    dyn2_d = np.full_like(t_sim, 3.5)

    dynamic_obstacles = [
        {
            "s_ref": dyn1_s,
            "d_ref": dyn1_d,
            "t_ref": t_sim,
            "length": 4.5,
            "width": 1.8,
            "safe_front": 6.0,
            "safe_back": 3.0,
            "safe_lat": 0.35,
        },
        {
            "s_ref": dyn2_s,
            "d_ref": dyn2_d,
            "t_ref": t_sim,
            "length": 4.6,
            "width": 1.9,
            "safe_front": 4.0,
            "safe_back": 2.5,
            "safe_lat": 0.25,
        },
    ]

    # 静态障碍（在时域内拉伸）
    static_obstacles = [
        (30.0, 37.0, -1.75, -1.0),
        (58.0, 66.0, -1.75, -1.0),
        (46.0, 53.0, 4.5, 5.25),
    ]

    # 生成 SSC 风格 corridor（更细时间切片）
    t_edges = np.arange(0.0, t_end + 0.25, 0.25)
    corridors = generate_spatiotemporal_corridor(
        t_ref=t_sim,
        ego_s_ref=ego_s_traj,
        ego_d_ref=ego_d_traj,
        t_edges=t_edges,
        lane_width=lane_width,
        t_start=0.0,
        v_min=v_min,
        v_max=v_max,
        static_obstacles=static_obstacles,
        dynamic_obstacles=dynamic_obstacles,
    )

    # 1) 速度边界与车道边界（半透明大面）
    draw_speed_limit_surfaces(ax, lane_d_min, lane_d_max, t_end, s0=0.0, v_min=v_min, v_max=v_max)
    draw_lane_boundary_surfaces(ax, 0.0, s_view_max, t_end, lane_d_min, lane_d_max)

    # 2) 静态障碍
    for s_min, s_max, d_min, d_max in static_obstacles:
        draw_cube(ax, s_min, s_max, d_min, d_max, 0.0, t_end, color='firebrick', alpha=0.22)
        draw_wire_cube(ax, s_min, s_max, d_min, d_max, 0.0, t_end, color='darkred', lw=1.2, alpha=0.95)

    # 3) 动态障碍
    draw_dynamic_vehicle_tube(
        ax, t_edges=t_edges, s_ref=dyn1_s, d_ref=dyn1_d, t_ref=t_sim,
        length=4.5, width=1.8, color='red', alpha=0.26
    )
    draw_dynamic_vehicle_tube(
        ax, t_edges=t_edges, s_ref=dyn2_s, d_ref=dyn2_d, t_ref=t_sim,
        length=4.6, width=1.9, color='tomato', alpha=0.24
    )

    # 4) SSC corridor 主体（浅绿色）
    corr_colors = plt.cm.Greens(np.linspace(0.25, 0.55, len(corridors)))
    for i, (s_min, s_max, d_min, d_max, t_min, t_max) in enumerate(corridors):
        draw_cube(ax, s_min, s_max, d_min, d_max, t_min, t_max, color=corr_colors[i], alpha=0.2)
        draw_wire_cube(ax, s_min, s_max, d_min, d_max, t_min, t_max, color='lightgreen', lw=0.8, alpha=0.95)

    # 5) Ego 轨迹 + 采样点
    ax.plot(ego_s_traj, ego_d_traj, t_sim, color='magenta', linewidth=3.5, alpha=0.95)
    key_t = np.arange(0.0, t_end + 1e-6, 0.5)
    for tk in key_t:
        s_k = np.interp(tk, t_sim, ego_s_traj)
        d_k = np.interp(tk, t_sim, ego_d_traj)
        ax.scatter(s_k, d_k, tk, color='deepskyblue', s=18, alpha=0.9)

    # 辅助轨迹（动态障碍中心线）
    ax.plot(dyn1_s, dyn1_d, t_sim, color='red', linestyle='--', linewidth=1.8, alpha=0.9)
    ax.plot(dyn2_s, dyn2_d, t_sim, color='orangered', linestyle='--', linewidth=1.6, alpha=0.85)

    # 坐标与范围
    ax.set_xlabel('s - Longitudinal Position (m)', fontsize=13)
    ax.set_ylabel('l / d - Lateral Position (m)', fontsize=13)
    ax.set_zlabel('t - Time (s)', fontsize=13)
    ax.set_title(
        'SSC-Style Spatio-Temporal Corridor with Lane/Speed Boundaries and Obstacles',
        fontsize=17,
        fontweight='bold',
        pad=8,
    )
    ax.set_xlim(0.0, s_view_max)
    ax.set_ylim(-2.6, 6.6)
    ax.set_zlim(0.0, t_end)

    # 保证 s 显著长于 l/t，但整体更饱满
    x_range = ax.get_xlim()[1] - ax.get_xlim()[0]
    y_range = ax.get_ylim()[1] - ax.get_ylim()[0]
    z_range = ax.get_zlim()[1] - ax.get_zlim()[0]
    # 增加横向/时间方向厚度，让主体不再像“细长条”
    ax.set_box_aspect((x_range, y_range * 2.6, z_range * 3.2))
    try:
        # 更大 focal_length = 更窄视角 = 更“放大”
        ax.set_proj_type('persp', focal_length=2.2)
    except TypeError:
        ax.set_proj_type('persp')
        if hasattr(ax, "dist"):
            ax.dist = 5.6

    ax.view_init(elev=21, azim=-60)
    ax.grid(True, alpha=0.22)

    legend_handles = [
        Line2D([0], [0], color='magenta', linewidth=3.0, label='Ego trajectory'),
        Patch(facecolor='lightgreen', edgecolor='lightgreen', alpha=0.22, label='SSC corridor cubes'),
        Patch(facecolor='firebrick', edgecolor='darkred', alpha=0.24, label='Static obstacles'),
        Patch(facecolor='red', edgecolor='black', alpha=0.24, label='Dynamic obstacles'),
        Line2D([0], [0], color='dodgerblue', linestyle=':', linewidth=2.2, label='Lane boundaries'),
        Line2D([0], [0], color='lightcoral', linestyle=':', linewidth=2.2, label='Speed limits'),
    ]
    ax.legend(
        handles=legend_handles,
        loc='upper left',
        fontsize=12,
        framealpha=0.95,
        bbox_to_anchor=(0.02, 0.98),
    )

    plt.subplots_adjust(left=0.00, right=1.00, bottom=0.00, top=0.95)
    plt.show()


def plot_vehicle_trajectories_2d():
    """绘制2D视图的车辆轨迹对比"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
    
    # 时间轴
    t_sim = np.linspace(0, 5, 500)
    
    # 车辆参数
    front_initial_s = 40.0
    front_speed = 12.0
    left_lane_d = 3.5
    
    # 使用多项式生成平滑轨迹
    ego_s_traj, ego_d_traj = smooth_lane_change_trajectory(t_sim)
    
    # 前车轨迹
    front_s_traj = front_initial_s + front_speed * t_sim
    front_d_traj = np.zeros_like(t_sim)
    
    # 计算速度和加速度
    dt = t_sim[1] - t_sim[0]
    ego_v_s = np.gradient(ego_s_traj, dt)  # 纵向速度
    ego_v_d = np.gradient(ego_d_traj, dt)  # 横向速度
    ego_a_s = np.gradient(ego_v_s, dt)     # 纵向加速度
    ego_a_d = np.gradient(ego_v_d, dt)     # 横向加速度
    
    # 子图1: s-t图（纵向位置-时间）
    ax1.plot(t_sim, ego_s_traj, 'b-', linewidth=3, label='Ego Longitudinal Position (Polynomial)')
    ax1.plot(t_sim, front_s_traj, 'r--', linewidth=2, label='Lead Vehicle Longitudinal Position')
    ax1.set_xlabel('Time t (s)', fontsize=11)
    ax1.set_ylabel('Longitudinal Position s (m)', fontsize=11)
    ax1.set_title('Longitudinal Motion Comparison', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # 标记关键时间点
    for t_key in [0, 1, 2, 3, 4, 5]:
        idx = int(t_key * (len(t_sim) - 1) / 5) if t_key < 5 else -1
        ax1.axvline(x=t_key, color='gray', linestyle=':', alpha=0.5)
        ax1.plot(t_key, ego_s_traj[idx], 'bo', markersize=6)
    
    # 子图2: d-t图（横向位置-时间）
    ax2.plot(t_sim, ego_d_traj, 'b-', linewidth=3, label='Ego Lateral Position (Polynomial)')
    ax2.plot(t_sim, front_d_traj, 'r--', linewidth=2, label='Lead Vehicle Lateral Position')
    ax2.axhline(y=0, color='k', linestyle=':', alpha=0.5, label='Right Lane Center')
    ax2.axhline(y=left_lane_d, color='k', linestyle=':', alpha=0.5, label='Left Lane Center')
    ax2.set_xlabel('Time t (s)', fontsize=11)
    ax2.set_ylabel('Lateral Position d (m)', fontsize=11)
    ax2.set_title('Lateral Motion Comparison (Smooth Lane Change)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # 标记变道区间
    ax2.axvspan(1.5, 3.5, alpha=0.2, color='yellow', label='Lane Change Period')
    for t_key in [0, 1, 2, 3, 4, 5]:
        ax2.axvline(x=t_key, color='gray', linestyle=':', alpha=0.5)
    
    # 子图3: 速度曲线
    ax3.plot(t_sim, ego_v_s, 'g-', linewidth=2, label='Longitudinal Velocity v_s')
    ax3.plot(t_sim, ego_v_d, 'orange', linewidth=2, label='Lateral Velocity v_d')
    ax3.set_xlabel('Time t (s)', fontsize=11)
    ax3.set_ylabel('Velocity (m/s)', fontsize=11)
    ax3.set_title('Ego Vehicle Velocity Profile', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    # 子图4: 加速度曲线
    ax4.plot(t_sim, ego_a_s, 'purple', linewidth=2, label='Longitudinal Acceleration a_s')
    ax4.plot(t_sim, ego_a_d, 'brown', linewidth=2, label='Lateral Acceleration a_d')
    ax4.set_xlabel('Time t (s)', fontsize=11)
    ax4.set_ylabel('Acceleration (m/s²)', fontsize=11)
    ax4.set_title('Ego Vehicle Acceleration Profile', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    ax4.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    
    # 添加总标题
    fig.suptitle('Detailed Analysis of Polynomial Trajectory Planning', fontsize=16, fontweight='bold')
    
    plt.tight_layout()
    plt.show()


def plot_trajectory_comparison():
    """对比线性变道和多项式变道的差异"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    t_sim = np.linspace(0, 5, 500)
    
    # 多项式轨迹
    ego_s_poly, ego_d_poly = smooth_lane_change_trajectory(t_sim)
    
    # 线性变道轨迹（原始方法）
    ego_s_linear = 15.0 * t_sim
    ego_d_linear = np.piecewise(
        t_sim,
        [t_sim <= 1.5, (t_sim > 1.5) & (t_sim <= 3.5), t_sim > 3.5],
        [0.0, lambda t: (t - 1.5) / (3.5 - 1.5) * 3.5, 3.5]
    )
    
    # 子图1: 轨迹对比
    ax1.plot(ego_s_poly, ego_d_poly, 'b-', linewidth=3, label='Polynomial Trajectory (Smooth)', alpha=0.8)
    ax1.plot(ego_s_linear, ego_d_linear, 'r--', linewidth=2, label='Linear Trajectory (Original)', alpha=0.8)
    ax1.set_xlabel('Longitudinal Position s (m)', fontsize=11)
    ax1.set_ylabel('Lateral Position d (m)', fontsize=11)
    ax1.set_title('Trajectory Path Comparison', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axhline(y=0, color='k', linestyle=':', alpha=0.5, label='Right Lane')
    ax1.axhline(y=3.5, color='k', linestyle=':', alpha=0.5, label='Left Lane')
    
    # 子图2: 横向速度对比
    dt = t_sim[1] - t_sim[0]
    v_d_poly = np.gradient(ego_d_poly, dt)
    v_d_linear = np.gradient(ego_d_linear, dt)
    
    ax2.plot(t_sim, v_d_poly, 'b-', linewidth=3, label='Polynomial Lateral Velocity', alpha=0.8)
    ax2.plot(t_sim, v_d_linear, 'r--', linewidth=2, label='Linear Lateral Velocity', alpha=0.8)
    ax2.set_xlabel('Time t (s)', fontsize=11)
    ax2.set_ylabel('Lateral Velocity v_d (m/s)', fontsize=11)
    ax2.set_title('Lateral Velocity Comparison', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax2.axvspan(1.5, 3.5, alpha=0.2, color='yellow', label='Lane Change Period')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 绘制3D时空走廊
    plot_3d_corridor_lane_change()
    
    # 绘制2D轨迹详细分析
    plot_vehicle_trajectories_2d()
    
    # 绘制轨迹对比图
    plot_trajectory_comparison()
