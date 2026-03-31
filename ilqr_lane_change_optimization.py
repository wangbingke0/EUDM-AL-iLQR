#!/usr/bin/env python3
"""
iLQR Trajectory Optimization for Lane Change Scenario
Optimize lane change trajectory with lane constraints and obstacle avoidance
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.optimize import minimize
import scipy.sparse as sp


def draw_cube(ax, x_min, x_max, y_min, y_max, t_min, t_max, color='lightblue', alpha=0.3):
    """Draw 3D cube representing spatio-temporal corridor"""
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


def smooth_lane_change_trajectory(t_array):
    """Generate smooth reference lane change trajectory"""
    s_traj = 15.0 * t_array  # Constant speed 15 m/s
    
    d_traj = np.zeros_like(t_array)
    for i, t in enumerate(t_array):
        if t <= 1.5:
            d_traj[i] = 0.0
        elif t >= 3.5:
            d_traj[i] = 3.5
        else:
            # Smooth lane change using 5th order polynomial
            tau = (t - 1.5) / (3.5 - 1.5)  # Normalized time [0,1]
            d_traj[i] = 3.5 * (10*tau**3 - 15*tau**4 + 6*tau**5)
    
    return s_traj, d_traj


def get_lead_vehicle_trajectory(t_array, front_initial_s=40.0, front_speed=12.0):
    """Get lead vehicle trajectory"""
    front_s_traj = front_initial_s + front_speed * t_array
    front_d_traj = np.zeros_like(t_array)
    return front_s_traj, front_d_traj


class ILQROptimizer:
    """Simple iLQR optimizer for trajectory optimization"""
    
    def __init__(self, N, dt=0.1):
        """
        Initialize iLQR optimizer
        N: number of time steps
        dt: time step
        """
        self.N = N
        self.dt = dt
        
    def cost_function(self, x, x_ref, u_ref, lead_s, lead_d, Q, R, Qf, lane_bounds, 
                     obstacle_margin=3.0, lateral_distance_weight=5000.0):
        """
        Compute total cost
        x: state trajectory [s0, d0, s1, d1, ..., sN, dN]
        x_ref: reference trajectory [s_ref, d_ref] for each time step
        u_ref: reference control (not used in this simple version)
        lead_s, lead_d: lead vehicle positions
        Q: state cost weight matrix
        R: control cost weight matrix
        Qf: terminal cost weight matrix
        lane_bounds: [d_min, d_max] lane boundaries
        obstacle_margin: minimum distance to lead vehicle
        lateral_distance_weight: weight for lateral distance penalty (encourages larger lateral separation)
        """
        cost = 0.0
        
        # Reshape x to [N+1, 2] where columns are [s, d]
        states = x.reshape(self.N + 1, 2)
        
        for k in range(self.N + 1):
            s_k = states[k, 0]
            d_k = states[k, 1]
            
            # Reference tracking cost
            if k < len(x_ref[0]):
                s_ref_k = x_ref[0][k]
                d_ref_k = x_ref[1][k]
                state_error = np.array([s_k - s_ref_k, d_k - d_ref_k])
                
                if k < self.N:
                    cost += state_error.T @ Q @ state_error
                else:
                    # Terminal cost
                    cost += state_error.T @ Qf @ state_error
            
            # Lane constraint violation (soft constraint)
            d_min, d_max = lane_bounds
            if d_k < d_min:
                cost += 1000 * (d_min - d_k)**2
            elif d_k > d_max:
                cost += 1000 * (d_k - d_max)**2
            
            # Obstacle avoidance cost (soft constraint)
            if k < len(lead_s):
                dist_s = s_k - lead_s[k]
                dist_d = d_k - lead_d[k]
                dist = np.sqrt(dist_s**2 + dist_d**2)
                
                # 1. Total distance penalty (existing)
                if dist < obstacle_margin:
                    # Penalty increases as distance decreases
                    penalty = 10000 * (obstacle_margin - dist)**2 / (dist + 0.1)
                    cost += penalty
                
                # 2. Lateral distance penalty (NEW) - encourages larger lateral separation
                # This is especially important when ego vehicle is behind or near the lead vehicle
                abs_lateral_dist = abs(dist_d)
                
                # When ego is behind lead vehicle (dist_s < 0) or close longitudinally (|dist_s| < 20m)
                # Strongly encourage larger lateral separation
                if dist_s < 20.0:  # Within 20m longitudinally
                    # Desired lateral separation: at least 3.5m (one lane width)
                    desired_lateral_sep = 3.5
                    
                    if abs_lateral_dist < desired_lateral_sep:
                        # Penalty increases quadratically as lateral distance decreases
                        lateral_penalty = lateral_distance_weight * (desired_lateral_sep - abs_lateral_dist)**2
                        cost += lateral_penalty
                    
                    # Additional penalty: encourage even larger separation when very close longitudinally
                    if dist_s < 10.0 and abs_lateral_dist < 4.0:
                        # Extra penalty for being too close laterally when longitudinally close
                        extra_penalty = lateral_distance_weight * 0.5 * (4.0 - abs_lateral_dist)**2
                        cost += extra_penalty
                
                # 3. Combined penalty: when both longitudinal and lateral distances are small
                if dist_s < 15.0 and abs_lateral_dist < 2.0:
                    # Very high penalty for being both longitudinally and laterally close
                    combined_penalty = 20000 * (15.0 - dist_s) * (2.0 - abs_lateral_dist)
                    cost += combined_penalty
        
        # Control cost (smoothness)
        for k in range(self.N):
            if k == 0:
                continue
            s_prev = states[k-1, 0]
            d_prev = states[k-1, 1]
            s_curr = states[k, 0]
            d_curr = states[k, 1]
            
            # Penalize large changes in velocity
            v_s = (s_curr - s_prev) / self.dt
            v_d = (d_curr - d_prev) / self.dt
            
            if k > 1:
                s_prev2 = states[k-2, 0]
                d_prev2 = states[k-2, 1]
                v_s_prev = (s_prev - s_prev2) / self.dt
                v_d_prev = (d_prev - d_prev2) / self.dt
                
                a_s = (v_s - v_s_prev) / self.dt
                a_d = (v_d - v_d_prev) / self.dt
                
                control = np.array([a_s, a_d])
                cost += control.T @ R @ control
        
        return cost
    
    def optimize(self, x_init, x_ref, lead_s, lead_d, lane_bounds, 
                 Q=None, R=None, Qf=None, obstacle_margin=3.0, 
                 lateral_distance_weight=5000.0, max_iter=100):
        """
        Optimize trajectory using iLQR
        lateral_distance_weight: weight for lateral distance penalty (higher = more emphasis on lateral separation)
        """
        if Q is None:
            Q = np.diag([1.0, 10.0])  # [s, d] tracking weights
        if R is None:
            R = np.diag([0.1, 1.0])  # [a_s, a_d] control weights
        if Qf is None:
            Qf = np.diag([10.0, 100.0])  # Terminal weights
        
        # Initial guess: reference trajectory
        x0 = np.zeros((self.N + 1) * 2)
        for k in range(self.N + 1):
            idx = k * 2
            if k < len(x_ref[0]):
                x0[idx] = x_ref[0][k]
                x0[idx + 1] = x_ref[1][k]
            else:
                # Extrapolate
                x0[idx] = x_ref[0][-1] + (k - len(x_ref[0]) + 1) * 15.0 * self.dt
                x0[idx + 1] = x_ref[1][-1]
        
        # Bounds: lane constraints
        bounds = []
        for k in range(self.N + 1):
            # s can be any positive value
            bounds.append((None, None))  # s
            # d must be within lane bounds
            bounds.append((lane_bounds[0] - 0.5, lane_bounds[1] + 0.5))  # d
        
        # Optimize
        result = minimize(
            self.cost_function,
            x0,
            args=(x_ref, None, lead_s, lead_d, Q, R, Qf, lane_bounds, 
                  obstacle_margin, lateral_distance_weight),
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': max_iter, 'disp': True}
        )
        
        if result.success:
            x_opt = result.x.reshape(self.N + 1, 2)
            return x_opt, result.fun
        else:
            print(f"Optimization warning: {result.message}")
            x_opt = result.x.reshape(self.N + 1, 2)
            return x_opt, result.fun


def plot_optimization_results(t_array, x_ref, x_opt, lead_s, lead_d, corridors=None):
    """Plot optimization results in 3D"""
    fig = plt.figure(figsize=(18, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    s_ref, d_ref = x_ref
    s_opt = x_opt[:, 0]
    d_opt = x_opt[:, 1]
    
    # Draw spatio-temporal corridors if provided
    if corridors is not None:
        colors = ['lightcoral', 'lightyellow', 'lightgreen', 'lightblue', 'plum']
        for i, (s_min, s_max, d_min, d_max, t_min, t_max) in enumerate(corridors):
            if i < len(colors):
                draw_cube(ax, s_min, s_max, d_min, d_max, t_min, t_max,
                         color=colors[i], alpha=0.2)
    
    # Plot reference trajectory
    ax.plot(s_ref, d_ref, t_array[:len(s_ref)], 'b--', linewidth=2.5, 
            label='Reference Trajectory', alpha=0.7)
    
    # Plot optimized trajectory
    t_opt = np.linspace(0, t_array[-1], len(s_opt))
    ax.plot(s_opt, d_opt, t_opt, 'g-', linewidth=3.5, 
            label='iLQR Optimized Trajectory', alpha=0.9)
    
    # Plot lead vehicle trajectory
    ax.plot(lead_s, lead_d, t_array[:len(lead_s)], 'r--', linewidth=2, 
            label='Lead Vehicle', alpha=0.7)
    
    # Mark key time points
    key_times = [0, 1, 2, 3, 4, 5]
    for t_key in key_times:
        if t_key <= t_array[-1]:
            # Reference trajectory markers
            idx_ref = int(t_key * len(s_ref) / t_array[-1]) if t_key < t_array[-1] else -1
            if idx_ref < len(s_ref):
                ax.scatter(s_ref[idx_ref], d_ref[idx_ref], t_key, 
                          c='blue', s=60, marker='o', alpha=0.6)
            
            # Optimized trajectory markers
            idx_opt = int(t_key * len(s_opt) / t_array[-1]) if t_key < t_array[-1] else -1
            if idx_opt < len(s_opt):
                ax.scatter(s_opt[idx_opt], d_opt[idx_opt], t_key, 
                          c='green', s=80, marker='s', alpha=0.8)
            
            # Lead vehicle markers
            idx_lead = int(t_key * len(lead_s) / t_array[-1]) if t_key < t_array[-1] else -1
            if idx_lead < len(lead_s):
                ax.scatter(lead_s[idx_lead], lead_d[idx_lead], t_key, 
                          c='red', s=60, marker='^', alpha=0.6)
    
    # Draw road boundaries
    road_length = 90
    ax.plot([0, road_length], [-1.75, -1.75], [0, 5], 'k-', linewidth=2, alpha=0.7, label='Road Boundary')
    ax.plot([0, road_length], [1.75, 1.75], [0, 5], 'k--', linewidth=1.5, alpha=0.6, label='Lane Line')
    ax.plot([0, road_length], [5.25, 5.25], [0, 5], 'k-', linewidth=2, alpha=0.7)
    
    # Set labels and title
    ax.set_xlabel('s - Longitudinal Position (m)', fontsize=12)
    ax.set_ylabel('d - Lateral Position (m)', fontsize=12)
    ax.set_zlabel('t - Time (s)', fontsize=12)
    ax.set_title('iLQR Trajectory Optimization: Lane Change with Constraints\n' +
                'Green: Optimized | Blue: Reference | Red: Lead Vehicle', 
                fontsize=14, fontweight='bold')
    
    # Set axis limits
    ax.set_xlim(0, 85)
    ax.set_ylim(-3, 7)
    ax.set_zlim(0, 5)
    
    # Legend
    ax.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax.view_init(elev=25, azim=-65)
    ax.grid(True, alpha=0.3)
    
    # Add info text
    ax.text2D(0.02, 0.98, 
              "Optimization Constraints:\n" +
              "• Lane bounds: d ∈ [-1.75, 5.25] m\n" +
              "• Obstacle avoidance: >3m total distance\n" +
              "• Lateral separation: Encourages >3.5m lateral distance\n" +
              "• Smoothness: Minimize acceleration\n" +
              "• Reference tracking: Follow reference trajectory", 
              transform=ax.transAxes, fontsize=9, verticalalignment='top',
              bbox=dict(boxstyle="round,pad=0.4", facecolor="lightcyan", alpha=0.9))
    
    plt.tight_layout()
    return fig


def frenet_to_cartesian(s_traj, d_traj, dt):
    """
    Convert Frenet coordinates (s, d) to Cartesian (x, y, yaw, v, a, kappa)
    Assuming reference line is straight along x-axis
    State: [x, y, yaw, v, a, kappa]
    """
    N = len(s_traj)
    x = np.zeros(N)
    y = np.zeros(N)
    yaw = np.zeros(N)
    v = np.zeros(N)
    a = np.zeros(N)
    kappa = np.zeros(N)
    
    # Convert s,d to x,y (assuming straight reference line along x-axis)
    x = s_traj.copy()
    y = d_traj.copy()
    
    # Calculate first derivatives (velocities)
    ds_dt = np.gradient(s_traj, dt)
    dd_dt = np.gradient(d_traj, dt)
    
    # Calculate second derivatives (accelerations)
    d2s_dt2 = np.gradient(ds_dt, dt)
    d2d_dt2 = np.gradient(dd_dt, dt)
    
    # Calculate third derivatives for curvature rate
    d3s_dt3 = np.gradient(d2s_dt2, dt)
    d3d_dt3 = np.gradient(d2d_dt2, dt)
    
    for i in range(N):
        # Velocity magnitude: v = sqrt((ds/dt)^2 + (dd/dt)^2)
        v[i] = np.sqrt(ds_dt[i]**2 + dd_dt[i]**2)
        
        # Yaw angle: yaw = atan2(dd/dt, ds/dt)
        if v[i] > 1e-6:
            yaw[i] = np.arctan2(dd_dt[i], ds_dt[i])
        else:
            yaw[i] = 0.0 if i == 0 else yaw[i-1]
        
        # Acceleration: a = (ds/dt * d2s/dt2 + dd/dt * d2d/dt2) / v
        if v[i] > 1e-6:
            a[i] = (ds_dt[i] * d2s_dt2[i] + dd_dt[i] * d2d_dt2[i]) / v[i]
        else:
            a[i] = 0.0
        
        # Curvature: kappa = |ds/dt * d2d/dt2 - dd/dt * d2s/dt2| / v^3
        if v[i] > 1e-6:
            numerator = abs(ds_dt[i] * d2d_dt2[i] - dd_dt[i] * d2s_dt2[i])
            kappa[i] = numerator / (v[i]**3)
        else:
            kappa[i] = 0.0
    
    return x, y, yaw, v, a, kappa


def calculate_controls(a_traj, kappa_traj, dt):
    """
    Calculate control inputs: da (jerk) and dkappa
    """
    N = len(a_traj)
    da = np.zeros(N)
    dkappa = np.zeros(N)
    
    for i in range(N):
        if i == 0:
            da[i] = (a_traj[i+1] - a_traj[i]) / dt if i+1 < N else 0
            dkappa[i] = (kappa_traj[i+1] - kappa_traj[i]) / dt if i+1 < N else 0
        elif i == N-1:
            da[i] = (a_traj[i] - a_traj[i-1]) / dt
            dkappa[i] = (kappa_traj[i] - kappa_traj[i-1]) / dt
        else:
            da[i] = (a_traj[i+1] - a_traj[i-1]) / (2*dt)
            dkappa[i] = (kappa_traj[i+1] - kappa_traj[i-1]) / (2*dt)
    
    return da, dkappa


def plot_controls(states_ref, states_opt, t_array, dt):
    """
    Plot control inputs (da, dkappa) for reference and optimized trajectories
    states_ref/opt: tuple of (x, y, yaw, v, a, kappa)
    """
    x_ref, y_ref, yaw_ref, v_ref, a_ref, kappa_ref = states_ref
    x_opt, y_opt, yaw_opt, v_opt, a_opt, kappa_opt = states_opt
    
    # Calculate controls
    da_ref, dkappa_ref = calculate_controls(a_ref, kappa_ref, dt)
    da_opt, dkappa_opt = calculate_controls(a_opt, kappa_opt, dt)
    
    # Create figure with subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
    
    # Subplot 1: Acceleration (a)
    ax1.plot(t_array[:len(a_ref)], a_ref, 'b--', linewidth=2, label='Reference', alpha=0.7)
    ax1.plot(t_array[:len(a_opt)], a_opt, 'g-', linewidth=3, label='Optimized', alpha=0.9)
    ax1.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax1.set_xlabel('Time t (s)', fontsize=11)
    ax1.set_ylabel('Acceleration a (m/s²)', fontsize=11)
    ax1.set_title('State: Acceleration', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Subplot 2: Control da (Jerk)
    ax2.plot(t_array[:len(da_ref)], da_ref, 'b--', linewidth=2, label='Reference da', alpha=0.7)
    ax2.plot(t_array[:len(da_opt)], da_opt, 'g-', linewidth=3, label='Optimized da', alpha=0.9)
    ax2.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax2.set_xlabel('Time t (s)', fontsize=11)
    ax2.set_ylabel('Control da - Jerk (m/s³)', fontsize=11)
    ax2.set_title('Control Input: da (Acceleration Rate)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Subplot 3: Curvature (kappa)
    ax3.plot(t_array[:len(kappa_ref)], kappa_ref, 'b--', linewidth=2, label='Reference', alpha=0.7)
    ax3.plot(t_array[:len(kappa_opt)], kappa_opt, 'g-', linewidth=3, label='Optimized', alpha=0.9)
    ax3.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax3.set_xlabel('Time t (s)', fontsize=11)
    ax3.set_ylabel('Curvature κ (1/m)', fontsize=11)
    ax3.set_title('State: Curvature', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Subplot 4: Control dkappa
    ax4.plot(t_array[:len(dkappa_ref)], dkappa_ref, 'b--', linewidth=2, label='Reference dkappa', alpha=0.7)
    ax4.plot(t_array[:len(dkappa_opt)], dkappa_opt, 'g-', linewidth=3, label='Optimized dkappa', alpha=0.9)
    ax4.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax4.set_xlabel('Time t (s)', fontsize=11)
    ax4.set_ylabel('Control dκ (1/m²)', fontsize=11)
    ax4.set_title('Control Input: dkappa (Curvature Rate)', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    fig.suptitle('Control Inputs: da and dkappa', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_sl_diagram_with_obstacles(s_ref, d_ref, s_opt, d_opt, lead_s, lead_d, t_array, 
                                   vehicle_length=4.5, vehicle_width=1.8, safety_margin=3.0):
    """
    Plot SL diagram (s-d plane) with obstacles and trajectory comparison
    Shows how optimized trajectory avoids obstacles better than reference
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
    
    # Left subplot: SL diagram with obstacles
    ax1.set_aspect('equal')
    
    # Draw road boundaries
    s_max = max(np.max(s_ref), np.max(s_opt), np.max(lead_s)) + 10
    ax1.axhspan(-1.75, 5.25, alpha=0.1, color='lightgray', label='Road Area')
    ax1.axhline(y=-1.75, color='k', linewidth=2, linestyle='-', alpha=0.7, label='Road Boundary')
    ax1.axhline(y=1.75, color='k', linewidth=1.5, linestyle='--', alpha=0.5, label='Lane Line')
    ax1.axhline(y=5.25, color='k', linewidth=2, linestyle='-', alpha=0.7)
    ax1.axhline(y=0, color='gray', linewidth=1, linestyle=':', alpha=0.5)
    ax1.axhline(y=3.5, color='gray', linewidth=1, linestyle=':', alpha=0.5)
    
    # Draw lead vehicle obstacle region (spatio-temporal occupancy)
    # For each time step, draw the vehicle's occupied region
    obstacle_polygons = []
    for i in range(len(lead_s)):
        s_center = lead_s[i]
        d_center = lead_d[i]
        
        # Vehicle bounding box
        s_min = s_center - vehicle_length / 2
        s_max = s_center + vehicle_length / 2
        d_min = d_center - vehicle_width / 2
        d_max = d_center + vehicle_width / 2
        
        # Draw vehicle rectangle
        if i == 0 or i == len(lead_s) - 1 or i % 5 == 0:  # Draw every 5th vehicle for clarity
            rect = plt.Rectangle((s_min, d_min), vehicle_length, vehicle_width,
                               facecolor='red', edgecolor='darkred', alpha=0.3, linewidth=1)
            ax1.add_patch(rect)
    
    # Draw expanded obstacle region (with safety margin) - this is the "danger zone"
    for i in range(0, len(lead_s), 2):  # Sample every 2nd point
        s_center = lead_s[i]
        d_center = lead_d[i]
        
        s_min = s_center - vehicle_length / 2 - safety_margin
        s_max = s_center + vehicle_length / 2 + safety_margin
        d_min = d_center - vehicle_width / 2 - safety_margin
        d_max = d_center + vehicle_width / 2 + safety_margin
        
        # Draw safety margin region
        rect_safety = plt.Rectangle((s_min, d_min), 
                                   s_max - s_min, d_max - d_min,
                                   facecolor='orange', edgecolor='orange', 
                                   alpha=0.15, linewidth=0.5, linestyle='--')
        ax1.add_patch(rect_safety)
    
    # Draw lead vehicle trajectory centerline
    ax1.plot(lead_s, lead_d, 'r-', linewidth=3, alpha=0.6, label='Lead Vehicle Center', zorder=1)
    
    # Draw reference trajectory (before optimization)
    ax1.plot(s_ref, d_ref, 'b--', linewidth=3, alpha=0.8, label='Reference Trajectory', zorder=3)
    
    # Draw optimized trajectory (after optimization)
    ax1.plot(s_opt, d_opt, 'g-', linewidth=4, alpha=0.9, label='Optimized Trajectory', zorder=4)
    
    # Mark start and end points
    ax1.plot(s_ref[0], d_ref[0], 'bo', markersize=12, label='Start', zorder=5)
    ax1.plot(s_ref[-1], d_ref[-1], 'b*', markersize=15, label='Reference End', zorder=5)
    ax1.plot(s_opt[0], d_opt[0], 'go', markersize=12, zorder=5)
    ax1.plot(s_opt[-1], d_opt[-1], 'g*', markersize=15, label='Optimized End', zorder=5)
    
    # Mark key points where trajectories are closest to obstacle
    min_dist_ref_idx = 0
    min_dist_opt_idx = 0
    min_dist_ref = float('inf')
    min_dist_opt = float('inf')
    
    for i in range(len(s_ref)):
        if i < len(lead_s):
            dist = np.sqrt((s_ref[i] - lead_s[i])**2 + (d_ref[i] - lead_d[i])**2)
            if dist < min_dist_ref:
                min_dist_ref = dist
                min_dist_ref_idx = i
    
    for i in range(len(s_opt)):
        idx_lead = int(i * len(lead_s) / len(s_opt)) if len(s_opt) > 0 else i
        if idx_lead < len(lead_s):
            dist = np.sqrt((s_opt[i] - lead_s[idx_lead])**2 + (d_opt[i] - lead_d[idx_lead])**2)
            if dist < min_dist_opt:
                min_dist_opt = dist
                min_dist_opt_idx = i
    
    # Highlight closest approach points
    ax1.plot(s_ref[min_dist_ref_idx], d_ref[min_dist_ref_idx], 'bx', 
            markersize=15, markeredgewidth=3, label=f'Ref Min Dist: {min_dist_ref:.2f}m', zorder=6)
    ax1.plot(s_opt[min_dist_opt_idx], d_opt[min_dist_opt_idx], 'gx', 
            markersize=15, markeredgewidth=3, label=f'Opt Min Dist: {min_dist_opt:.2f}m', zorder=6)
    
    ax1.set_xlabel('Longitudinal Position s (m)', fontsize=12)
    ax1.set_ylabel('Lateral Position d (m)', fontsize=12)
    ax1.set_title('SL Diagram: Trajectory Comparison with Obstacles\n' +
                 'Optimized trajectory avoids obstacles better', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=9, framealpha=0.9)
    ax1.set_xlim(0, s_max)
    ax1.set_ylim(-3, 7)
    
    # Right subplot: Distance to obstacle over time
    distances_ref = []
    distances_opt = []
    times_ref = []
    times_opt = []
    
    for i in range(len(s_ref)):
        if i < len(lead_s):
            dist = np.sqrt((s_ref[i] - lead_s[i])**2 + (d_ref[i] - lead_d[i])**2)
            distances_ref.append(dist)
            times_ref.append(t_array[i] if i < len(t_array) else i * 0.1)
    
    for i in range(len(s_opt)):
        idx_lead = int(i * len(lead_s) / len(s_opt)) if len(s_opt) > 0 else i
        if idx_lead < len(lead_s):
            dist = np.sqrt((s_opt[i] - lead_s[idx_lead])**2 + (d_opt[i] - lead_d[idx_lead])**2)
            distances_opt.append(dist)
            times_opt.append(t_array[i] if i < len(t_array) else i * 0.1)
    
    ax2.plot(times_ref, distances_ref, 'b--', linewidth=3, label='Reference Distance', alpha=0.8)
    ax2.plot(times_opt, distances_opt, 'g-', linewidth=4, label='Optimized Distance', alpha=0.9)
    ax2.axhline(y=safety_margin, color='r', linestyle='--', linewidth=2, 
               alpha=0.7, label=f'Safety Margin ({safety_margin}m)')
    ax2.fill_between(times_ref, 0, safety_margin, alpha=0.2, color='red', label='Danger Zone')
    
    # Highlight minimum distances
    ax2.plot(times_ref[min_dist_ref_idx], min_dist_ref, 'bx', 
            markersize=15, markeredgewidth=3, zorder=5)
    ax2.plot(times_opt[min_dist_opt_idx], min_dist_opt, 'gx', 
            markersize=15, markeredgewidth=3, zorder=5)
    
    ax2.set_xlabel('Time t (s)', fontsize=12)
    ax2.set_ylabel('Distance to Obstacle (m)', fontsize=12)
    ax2.set_title('Distance to Obstacle Over Time\n' +
                 f'Reference min: {min_dist_ref:.2f}m | Optimized min: {min_dist_opt:.2f}m', 
                 fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax2.set_ylim(0, max(max(distances_ref), max(distances_opt)) * 1.1)
    
    fig.suptitle('SL Diagram Analysis: Obstacle Avoidance Comparison', 
                fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_2d_comparison(t_array, x_ref, x_opt, lead_s, lead_d):
    """Plot 2D comparison of trajectories"""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 10))
    
    s_ref, d_ref = x_ref
    s_opt = x_opt[:, 0]
    d_opt = x_opt[:, 1]
    t_opt = np.linspace(0, t_array[-1], len(s_opt))
    
    # Subplot 1: s-t (longitudinal position vs time)
    ax1.plot(t_array[:len(s_ref)], s_ref, 'b--', linewidth=2, label='Reference', alpha=0.7)
    ax1.plot(t_opt, s_opt, 'g-', linewidth=3, label='Optimized', alpha=0.9)
    ax1.plot(t_array[:len(lead_s)], lead_s, 'r--', linewidth=2, label='Lead Vehicle', alpha=0.7)
    ax1.set_xlabel('Time t (s)', fontsize=11)
    ax1.set_ylabel('Longitudinal Position s (m)', fontsize=11)
    ax1.set_title('Longitudinal Motion Comparison', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Subplot 2: d-t (lateral position vs time)
    ax1_twin = ax1.twinx()
    ax1_twin.plot(t_array[:len(d_ref)], d_ref, 'b--', linewidth=2, alpha=0.5, linestyle=':')
    ax1_twin.plot(t_opt, d_opt, 'g-', linewidth=2, alpha=0.5, linestyle=':')
    ax1_twin.set_ylabel('Lateral Position d (m)', fontsize=11, color='gray')
    ax1_twin.tick_params(axis='y', labelcolor='gray')
    
    ax2.plot(t_array[:len(d_ref)], d_ref, 'b--', linewidth=2, label='Reference', alpha=0.7)
    ax2.plot(t_opt, d_opt, 'g-', linewidth=3, label='Optimized', alpha=0.9)
    ax2.plot(t_array[:len(lead_d)], lead_d, 'r--', linewidth=2, label='Lead Vehicle', alpha=0.7)
    ax2.axhline(y=0, color='k', linestyle=':', alpha=0.5, label='Right Lane Center')
    ax2.axhline(y=3.5, color='k', linestyle=':', alpha=0.5, label='Left Lane Center')
    ax2.axhspan(-1.75, 5.25, alpha=0.1, color='yellow', label='Lane Bounds')
    ax2.set_xlabel('Time t (s)', fontsize=11)
    ax2.set_ylabel('Lateral Position d (m)', fontsize=11)
    ax2.set_title('Lateral Motion Comparison', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Subplot 3: Distance to lead vehicle
    distances_ref = []
    distances_opt = []
    for i in range(min(len(s_ref), len(lead_s))):
        dist_ref = np.sqrt((s_ref[i] - lead_s[i])**2 + (d_ref[i] - lead_d[i])**2)
        distances_ref.append(dist_ref)
    
    for i in range(min(len(s_opt), len(lead_s))):
        idx_lead = int(i * len(lead_s) / len(s_opt)) if len(s_opt) > 0 else i
        if idx_lead < len(lead_s):
            dist_opt = np.sqrt((s_opt[i] - lead_s[idx_lead])**2 + (d_opt[i] - lead_d[idx_lead])**2)
            distances_opt.append(dist_opt)
    
    ax3.plot(t_array[:len(distances_ref)], distances_ref, 'b--', linewidth=2, label='Reference Distance', alpha=0.7)
    ax3.plot(t_opt[:len(distances_opt)], distances_opt, 'g-', linewidth=3, label='Optimized Distance', alpha=0.9)
    ax3.axhline(y=3.0, color='r', linestyle='--', alpha=0.5, label='Safety Margin (3m)')
    ax3.set_xlabel('Time t (s)', fontsize=11)
    ax3.set_ylabel('Distance to Lead Vehicle (m)', fontsize=11)
    ax3.set_title('Obstacle Avoidance: Distance to Lead Vehicle', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # Subplot 4: Curvature comparison (smoothness indicator)
    # Calculate curvature for both trajectories
    kappa_ref = []
    kappa_opt = []
    for i in range(1, len(s_ref)-1):
        # Simple curvature approximation: |d2d/ds2| / (1 + (dd/ds)^2)^(3/2)
        ds = s_ref[i+1] - s_ref[i-1]
        dd = d_ref[i+1] - d_ref[i-1]
        d2d = d_ref[i+1] - 2*d_ref[i] + d_ref[i-1]
        if abs(ds) > 1e-6:
            dd_ds = dd / ds
            d2d_ds2 = d2d / (ds**2)
            kappa = abs(d2d_ds2) / ((1 + dd_ds**2)**1.5) if abs(ds) > 1e-6 else 0
            kappa_ref.append(kappa)
        else:
            kappa_ref.append(0)
    
    for i in range(1, len(s_opt)-1):
        ds = s_opt[i+1] - s_opt[i-1]
        dd = d_opt[i+1] - d_opt[i-1]
        d2d = d_opt[i+1] - 2*d_opt[i] + d_opt[i-1]
        if abs(ds) > 1e-6:
            dd_ds = dd / ds
            d2d_ds2 = d2d / (ds**2)
            kappa = abs(d2d_ds2) / ((1 + dd_ds**2)**1.5) if abs(ds) > 1e-6 else 0
            kappa_opt.append(kappa)
        else:
            kappa_opt.append(0)
    
    t_kappa_ref = t_array[1:len(kappa_ref)+1] if len(kappa_ref) < len(t_array) else t_array[1:-1]
    t_kappa_opt = t_array[1:len(kappa_opt)+1] if len(kappa_opt) < len(t_array) else t_array[1:-1]
    
    ax4.plot(t_kappa_ref[:len(kappa_ref)], kappa_ref, 'b--', linewidth=2, 
            label='Reference Curvature', alpha=0.7)
    ax4.plot(t_kappa_opt[:len(kappa_opt)], kappa_opt, 'g-', linewidth=3, 
            label='Optimized Curvature', alpha=0.9)
    ax4.axhline(y=0, color='k', linestyle=':', alpha=0.3)
    ax4.set_xlabel('Time t (s)', fontsize=11)
    ax4.set_ylabel('Curvature κ (1/m)', fontsize=11)
    ax4.set_title('Trajectory Smoothness: Curvature Comparison', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    fig.suptitle('iLQR Optimization Results: Detailed Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig


def main():
    """Main function"""
    print("=" * 60)
    print("iLQR Trajectory Optimization for Lane Change Scenario")
    print("=" * 60)
    print()
    
    # Time parameters
    t_total = 5.0  # Total time
    dt = 0.1  # Time step
    N = int(t_total / dt)  # Number of time steps
    t_array = np.linspace(0, t_total, N + 1)
    
    print(f"Time parameters:")
    print(f"  Total time: {t_total} s")
    print(f"  Time step: {dt} s")
    print(f"  Number of steps: {N}")
    print()
    
    # Generate reference trajectory
    print("Generating reference trajectory...")
    s_ref, d_ref = smooth_lane_change_trajectory(t_array)
    x_ref = [s_ref, d_ref]
    print(f"  Reference trajectory: {len(s_ref)} points")
    print()
    
    # Generate lead vehicle trajectory
    print("Generating lead vehicle trajectory...")
    lead_s, lead_d = get_lead_vehicle_trajectory(t_array, front_initial_s=40.0, front_speed=12.0)
    print(f"  Lead vehicle trajectory: {len(lead_s)} points")
    print()
    
    # Define constraints
    lane_bounds = [-1.75, 5.25]  # Road boundaries
    obstacle_margin = 3.0  # Minimum distance to lead vehicle (m)
    lateral_distance_weight = 5000.0  # Weight for lateral distance penalty (higher = more emphasis on lateral separation)
    
    print("Constraints:")
    print(f"  Lane bounds: d ∈ [{lane_bounds[0]}, {lane_bounds[1]}] m")
    print(f"  Obstacle margin: {obstacle_margin} m")
    print(f"  Lateral distance weight: {lateral_distance_weight}")
    print(f"    (Encourages larger lateral separation from lead vehicle)")
    print()
    
    # Create optimizer
    print("Initializing iLQR optimizer...")
    optimizer = ILQROptimizer(N, dt)
    
    # Optimize trajectory
    print("Optimizing trajectory...")
    print("  This may take a moment...")
    print("  Cost function includes:")
    print("    - Reference tracking")
    print("    - Lane boundary constraints")
    print("    - Obstacle avoidance (total distance)")
    print("    - Lateral distance penalty (NEW: encourages larger lateral separation)")
    print("    - Trajectory smoothness")
    x_opt, final_cost = optimizer.optimize(
        x_init=None,
        x_ref=x_ref,
        lead_s=lead_s,
        lead_d=lead_d,
        lane_bounds=lane_bounds,
        obstacle_margin=obstacle_margin,
        lateral_distance_weight=lateral_distance_weight,
        max_iter=200
    )
    
    print()
    print(f"Optimization complete!")
    print(f"  Final cost: {final_cost:.2f}")
    print()
    
    # Define corridors for visualization
    corridors = [
        (0.0, 20.0, -1.2, 1.2, 0.0, 1.0),
        (10.0, 35.0, -1.0, 2.0, 1.0, 2.0),
        (20.0, 50.0, -0.5, 4.0, 2.0, 3.0),
        (35.0, 65.0, 1.0, 4.5, 3.0, 4.0),
        (50.0, 80.0, 2.5, 4.5, 4.0, 5.0),
    ]
    
    # Plot results
    print("Generating visualizations...")
    fig1 = plot_optimization_results(t_array, x_ref, x_opt, lead_s, lead_d, corridors)
    fig2 = plot_2d_comparison(t_array, x_ref, x_opt, lead_s, lead_d)
    
    # Extract trajectory data for SL diagram
    s_ref, d_ref = x_ref
    s_opt = x_opt[:, 0]
    d_opt = x_opt[:, 1]
    
    # Plot SL diagram with obstacles
    print("Plotting SL diagram with obstacles...")
    fig_sl = plot_sl_diagram_with_obstacles(s_ref, d_ref, s_opt, d_opt, lead_s, lead_d, t_array,
                                            vehicle_length=4.5, vehicle_width=1.8, safety_margin=3.0)
    
    # Calculate states and controls
    print("Calculating states and controls...")
    
    # Convert Frenet to Cartesian states
    states_ref = frenet_to_cartesian(s_ref, d_ref, dt)
    states_opt = frenet_to_cartesian(s_opt, d_opt, dt)
    
    print(f"  Reference states: x, y, yaw, v, a, kappa calculated")
    print(f"  Optimized states: x, y, yaw, v, a, kappa calculated")
    
    # Plot control inputs
    print("Plotting control inputs...")
    fig3 = plot_controls(states_ref, states_opt, t_array, dt)
    
    print()
    print("=" * 60)
    print("Visualization Complete!")
    print("=" * 60)
    print()
    print("Close windows to exit.")
    
    plt.show()


if __name__ == '__main__':
    main()

