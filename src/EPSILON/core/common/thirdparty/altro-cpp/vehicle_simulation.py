#!/usr/bin/env python3
"""
Vehicle Simulation Visualization
Creates an animated simulation of vehicle trajectory with detailed vehicle model.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, FancyArrow, Rectangle, FancyBboxPatch
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PatchCollection
import matplotlib.gridspec as gridspec
import os
import sys

# Set style
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['axes.facecolor'] = 'white'
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.labelcolor'] = '#333333'
plt.rcParams['text.color'] = '#333333'
plt.rcParams['xtick.color'] = '#333333'
plt.rcParams['ytick.color'] = '#333333'
plt.rcParams['grid.color'] = '#cccccc'
plt.rcParams['grid.alpha'] = 0.5


def load_trajectory(filename):
    """Load trajectory data from CSV file."""
    data = {
        'x': [], 'y': [], 'yaw': [], 'v': [], 'a': [], 'kappa': [],
        'da': [], 'dkappa': [], 't': [],
        'goal': None,
        'obstacles': []  # List of (x, y, r) tuples
    }
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            if line.startswith('GOAL,'):
                parts = line.split(',')[1:]
                data['goal'] = [float(p) for p in parts]
            elif line.startswith('OBS,'):
                parts = line.split(',')[1:]
                if len(parts) >= 3:
                    data['obstacles'].append((float(parts[0]), float(parts[1]), float(parts[2])))
            else:
                parts = line.split(',')
                if len(parts) >= 9:
                    data['x'].append(float(parts[0]))
                    data['y'].append(float(parts[1]))
                    data['yaw'].append(float(parts[2]))
                    data['v'].append(float(parts[3]))
                    data['a'].append(float(parts[4]))
                    data['kappa'].append(float(parts[5]))
                    data['da'].append(float(parts[6]))
                    data['dkappa'].append(float(parts[7]))
                    data['t'].append(float(parts[8]))
    
    for key in data:
        if key not in ['goal', 'obstacles'] and data[key]:
            data[key] = np.array(data[key])
    
    return data


class VehicleDrawer:
    """Draw a detailed vehicle with body and wheels."""
    
    def __init__(self, ax, length=2.5, width=1.2, wheel_length=0.4, wheel_width=0.2):
        self.ax = ax
        self.length = length
        self.width = width
        self.wheel_length = wheel_length
        self.wheel_width = wheel_width
        
        # Create vehicle body
        self.body = Polygon(np.zeros((4, 2)), closed=True, 
                           facecolor='#e94560', edgecolor='#333333', 
                           linewidth=2, zorder=10, alpha=0.9)
        ax.add_patch(self.body)
        
        # Create windshield
        self.windshield = Polygon(np.zeros((4, 2)), closed=True,
                                  facecolor='#4fc3f7', edgecolor='#333333',
                                  linewidth=1, zorder=11, alpha=0.8)
        ax.add_patch(self.windshield)
        
        # Create wheels (4 wheels)
        self.wheels = []
        for _ in range(4):
            wheel = Polygon(np.zeros((4, 2)), closed=True,
                           facecolor='#2d2d2d', edgecolor='#333333',
                           linewidth=1, zorder=9)
            ax.add_patch(wheel)
            self.wheels.append(wheel)
        
        # Heading arrow
        self.arrow = None
        
    def _rotate_points(self, points, angle, center):
        """Rotate points around a center."""
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return (points - center) @ R.T + center
    
    def update(self, x, y, yaw, steering_angle=0):
        """Update vehicle position and orientation."""
        # Vehicle body corners (rear-axle centered)
        rear_overhang = self.length * 0.2
        front_overhang = self.length * 0.8
        half_width = self.width / 2
        
        body_corners = np.array([
            [-rear_overhang, -half_width],
            [front_overhang, -half_width],
            [front_overhang, half_width],
            [-rear_overhang, half_width]
        ])
        
        # Rotate and translate body
        center = np.array([x, y])
        body_rotated = self._rotate_points(body_corners + center, yaw, center)
        self.body.set_xy(body_rotated)
        
        # Windshield (front part of vehicle)
        ws_start = front_overhang * 0.5
        ws_end = front_overhang * 0.85
        ws_width = half_width * 0.8
        windshield_corners = np.array([
            [ws_start, -ws_width],
            [ws_end, -ws_width * 0.7],
            [ws_end, ws_width * 0.7],
            [ws_start, ws_width]
        ])
        windshield_rotated = self._rotate_points(windshield_corners + center, yaw, center)
        self.windshield.set_xy(windshield_rotated)
        
        # Wheel positions (relative to rear axle center)
        wheelbase = self.length * 0.6
        track = self.width * 0.9
        
        wheel_positions = [
            (0, -track/2),           # Rear left
            (0, track/2),            # Rear right
            (wheelbase, -track/2),   # Front left
            (wheelbase, track/2),    # Front right
        ]
        
        for i, (wx, wy) in enumerate(wheel_positions):
            # Wheel corners
            wl, ww = self.wheel_length / 2, self.wheel_width / 2
            wheel_corners = np.array([
                [-wl, -ww],
                [wl, -ww],
                [wl, ww],
                [-wl, ww]
            ])
            
            # Front wheels have steering angle
            wheel_yaw = yaw
            if i >= 2:  # Front wheels
                wheel_yaw += steering_angle
            
            # Position wheel
            wheel_center = center + np.array([
                wx * np.cos(yaw) - wy * np.sin(yaw),
                wx * np.sin(yaw) + wy * np.cos(yaw)
            ])
            
            wheel_rotated = self._rotate_points(wheel_corners + wheel_center, wheel_yaw, wheel_center)
            self.wheels[i].set_xy(wheel_rotated)


def create_simulation(data, output_file=None, show=True):
    """Create vehicle simulation animation."""
    
    # Create figure with white theme
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    gs = gridspec.GridSpec(2, 3, figure=fig, height_ratios=[3, 1], 
                           hspace=0.25, wspace=0.3)
    
    # Main simulation view
    ax_main = fig.add_subplot(gs[0, :2])
    ax_main.set_facecolor('white')
    
    # State panels
    ax_vel = fig.add_subplot(gs[0, 2])
    ax_vel.set_facecolor('white')
    ax_ctrl = fig.add_subplot(gs[1, :])
    ax_ctrl.set_facecolor('white')
    
    # Setup main axis
    margin = 3
    x_min, x_max = min(data['x']) - margin, max(data['x']) + margin
    y_min, y_max = min(data['y']) - margin, max(data['y']) + margin
    
    # Make aspect ratio equal but with some flexibility
    x_range = x_max - x_min
    y_range = y_max - y_min
    if x_range > y_range * 1.5:
        y_center = (y_max + y_min) / 2
        y_min = y_center - x_range / 3
        y_max = y_center + x_range / 3
    elif y_range > x_range * 1.5:
        x_center = (x_max + x_min) / 2
        x_min = x_center - y_range / 3
        x_max = x_center + y_range / 3
    
    ax_main.set_xlim(x_min, x_max)
    ax_main.set_ylim(y_min, y_max)
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.3, color='#cccccc')
    ax_main.set_xlabel('X Position [m]', fontsize=12)
    ax_main.set_ylabel('Y Position [m]', fontsize=12)
    ax_main.set_title('🚗 Vehicle Trajectory Simulation', fontsize=16, fontweight='bold', color='#e94560')
    
    # Draw road/background elements
    # Reference trajectory (light)
    ax_main.plot(data['x'], data['y'], '-', color='#4fc3f7', alpha=0.3, linewidth=8, 
                 label='Planned Path', zorder=1)
    ax_main.plot(data['x'], data['y'], '--', color='#666666', alpha=0.5, linewidth=1, zorder=2)
    
    # Draw obstacles (if any)
    for i, obs in enumerate(data['obstacles']):
        obs_x, obs_y, obs_r = obs
        # Draw obstacle danger zone (outer circle)
        danger_zone = Circle((obs_x, obs_y), obs_r, 
                             facecolor='#ff0000', alpha=0.2, 
                             edgecolor='#ff0000', linewidth=2, linestyle='--',
                             label='Obstacle' if i == 0 else '', zorder=3)
        ax_main.add_patch(danger_zone)
        
        # Draw obstacle vehicle body (inner rectangle)
        obs_length, obs_width = 2.0, 1.0
        corners = np.array([
            [-obs_length * 0.3, -obs_width / 2],
            [obs_length * 0.7, -obs_width / 2],
            [obs_length * 0.7, obs_width / 2],
            [-obs_length * 0.3, obs_width / 2],
        ])
        corners = corners + np.array([obs_x, obs_y])
        obs_body = Polygon(corners, closed=True, 
                          facecolor='#ff4444', edgecolor='#333333',
                          linewidth=2, zorder=4, alpha=0.9)
        ax_main.add_patch(obs_body)
        
        # Add warning text
        ax_main.annotate('OBSTACLE', (obs_x, obs_y + obs_r + 0.5), 
                        ha='center', fontsize=9, color='#ff6666', fontweight='bold')
    
    # Start marker
    ax_main.plot(data['x'][0], data['y'][0], 'o', color='#00ff88', markersize=15, 
                 label='Start', zorder=5)
    ax_main.annotate('START', (data['x'][0], data['y'][0]), 
                     textcoords="offset points", xytext=(0, 15),
                     ha='center', fontsize=10, color='#00ff88', fontweight='bold')
    
    # Goal marker
    if data['goal'] is not None:
        ax_main.plot(data['goal'][0], data['goal'][1], '*', color='#ffff00', 
                     markersize=20, label='Goal', zorder=5)
        ax_main.annotate('GOAL', (data['goal'][0], data['goal'][1]), 
                         textcoords="offset points", xytext=(0, 15),
                         ha='center', fontsize=10, color='#ffff00', fontweight='bold')
    
    ax_main.legend(loc='upper left', facecolor='white', edgecolor='#333333')
    
    # Create vehicle
    vehicle = VehicleDrawer(ax_main, length=2.0, width=1.0)
    
    # Trail line (driven path)
    trail, = ax_main.plot([], [], '-', color='#e94560', linewidth=3, alpha=0.8, zorder=3)
    
    # Current position marker
    pos_marker, = ax_main.plot([], [], 'o', color='#333333', markersize=5, zorder=15)
    
    # Velocity gauge
    ax_vel.set_xlim(-1.5, 1.5)
    ax_vel.set_ylim(-1.5, 1.5)
    ax_vel.set_aspect('equal')
    ax_vel.axis('off')
    ax_vel.set_title('📊 Vehicle State', fontsize=12, color='#e94560')
    
    # Draw gauge background
    theta = np.linspace(0, np.pi, 100)
    ax_vel.plot(np.cos(theta), np.sin(theta), '-', color='#666666', linewidth=10)
    
    # Gauge ticks
    v_max = max(abs(data['v'].min()), abs(data['v'].max())) * 1.2
    for i, v in enumerate(np.linspace(0, v_max, 6)):
        angle = np.pi * (1 - v / v_max)
        x_tick = 0.85 * np.cos(angle)
        y_tick = 0.85 * np.sin(angle)
        ax_vel.plot([0.75 * np.cos(angle), x_tick], [0.75 * np.sin(angle), y_tick], 
                    '-', color='#333333', linewidth=2)
        ax_vel.text(1.05 * np.cos(angle), 1.05 * np.sin(angle), f'{v:.1f}', 
                    ha='center', va='center', fontsize=8, color='#333333')
    
    # Gauge needle
    gauge_needle, = ax_vel.plot([0, 0], [0, 0.7], '-', color='#e94560', linewidth=3)
    
    # State text displays
    state_text = ax_vel.text(0, -0.3, '', ha='center', va='center', fontsize=11, 
                              color='#333333', family='monospace')
    time_text = ax_vel.text(0, -0.6, '', ha='center', va='center', fontsize=14, 
                            color='#4fc3f7', fontweight='bold')
    
    # Control inputs plot
    t_data = data['t']
    ax_ctrl.set_xlim(0, t_data[-1])
    ctrl_max = max(abs(data['da']).max(), abs(data['dkappa']).max()) * 1.2
    ax_ctrl.set_ylim(-ctrl_max, ctrl_max)
    ax_ctrl.set_xlabel('Time [s]', fontsize=10)
    ax_ctrl.set_ylabel('Control Input', fontsize=10)
    ax_ctrl.set_title('🎮 Control Inputs', fontsize=12, color='#e94560')
    ax_ctrl.grid(True, alpha=0.3)
    
    # Full control trajectories (background)
    ax_ctrl.plot(t_data[:-1], data['da'][:-1], '-', color='#ff6b6b', alpha=0.3, linewidth=1, label='Jerk (da)')
    ax_ctrl.plot(t_data[:-1], data['dkappa'][:-1], '-', color='#4ecdc4', alpha=0.3, linewidth=1, label='dκ')
    
    # Current control lines
    ctrl_line_da, = ax_ctrl.plot([], [], '-', color='#ff6b6b', linewidth=2)
    ctrl_line_dk, = ax_ctrl.plot([], [], '-', color='#4ecdc4', linewidth=2)
    ctrl_marker, = ax_ctrl.plot([], [], 'o', color='#333333', markersize=8, zorder=10)
    ax_ctrl.legend(loc='upper right', facecolor='white', edgecolor='#333333')
    
    # Time indicator line
    time_line = ax_ctrl.axvline(x=0, color='#e94560', linestyle='--', linewidth=2, alpha=0.8)
    
    def init():
        vehicle.update(data['x'][0], data['y'][0], data['yaw'][0])
        trail.set_data([], [])
        pos_marker.set_data([], [])
        gauge_needle.set_data([0, 0], [0, 0.7])
        state_text.set_text('')
        time_text.set_text('')
        ctrl_line_da.set_data([], [])
        ctrl_line_dk.set_data([], [])
        ctrl_marker.set_data([], [])
        time_line.set_xdata([0])
        return (vehicle.body, vehicle.windshield, *vehicle.wheels, trail, pos_marker,
                gauge_needle, state_text, time_text, ctrl_line_da, ctrl_line_dk, 
                ctrl_marker, time_line)
    
    def update(frame):
        # Update vehicle position
        x, y, yaw = data['x'][frame], data['y'][frame], data['yaw'][frame]
        v, a, kappa = data['v'][frame], data['a'][frame], data['kappa'][frame]
        t = data['t'][frame]
        
        # Compute steering angle from curvature (approximate)
        wheelbase = 2.0 * 0.6
        steering = np.arctan(kappa * wheelbase) if abs(kappa) < 10 else 0
        
        vehicle.update(x, y, yaw, steering)
        
        # Update trail
        trail.set_data(data['x'][:frame+1], data['y'][:frame+1])
        pos_marker.set_data([x], [y])
        
        # Update velocity gauge
        v_norm = abs(v) / v_max if v_max > 0 else 0
        angle = np.pi * (1 - v_norm)
        gauge_needle.set_data([0, 0.65 * np.cos(angle)], [0, 0.65 * np.sin(angle)])
        
        # Update state text
        state_text.set_text(f'v: {v:.2f} m/s\na: {a:.2f} m/s²\nκ: {kappa:.4f} 1/m')
        time_text.set_text(f'⏱ t = {t:.2f} s')
        
        # Update control plots
        ctrl_line_da.set_data(t_data[:frame+1], data['da'][:frame+1])
        ctrl_line_dk.set_data(t_data[:frame+1], data['dkappa'][:frame+1])
        if frame < len(data['da']):
            ctrl_marker.set_data([t], [data['da'][frame]])
        time_line.set_xdata([t])
        
        return (vehicle.body, vehicle.windshield, *vehicle.wheels, trail, pos_marker,
                gauge_needle, state_text, time_text, ctrl_line_da, ctrl_line_dk,
                ctrl_marker, time_line)
    
    # Create animation
    frames = len(data['x'])
    interval = int(1000 * (data['t'][-1] / frames))  # Adjust speed to match real time
    interval = max(20, min(interval, 100))  # Clamp between 20-100ms
    
    anim = FuncAnimation(fig, update, frames=frames, init_func=init,
                         blit=True, interval=interval, repeat=True)
    
    plt.tight_layout()
    
    if output_file:
        print(f"Saving animation to {output_file}...")
        anim.save(output_file, writer='pillow', fps=30, dpi=100)
        print(f"Animation saved to: {output_file}")
    
    if show:
        plt.show()
    
    return anim


def main():
    final_file = "/tmp/vehicle_final.csv"
    
    if not os.path.exists(final_file):
        print(f"Error: Trajectory file not found: {final_file}")
        print("Please run vehicle_trajectory_demo first to generate trajectory data.")
        return
    
    data = load_trajectory(final_file)
    
    if len(data['x']) == 0:
        print("Error: Empty trajectory data")
        return
    
    print("=" * 50)
    print("🚗 Vehicle Trajectory Simulation")
    print("=" * 50)
    print(f"Trajectory points: {len(data['x'])}")
    print(f"Duration: {data['t'][-1]:.2f} s")
    print(f"Start: ({data['x'][0]:.2f}, {data['y'][0]:.2f})")
    print(f"End: ({data['x'][-1]:.2f}, {data['y'][-1]:.2f})")
    if data['goal']:
        print(f"Goal: ({data['goal'][0]:.2f}, {data['goal'][1]:.2f})")
    print("=" * 50)
    
    # Check for --save flag
    output_file = None
    if len(sys.argv) > 1 and sys.argv[1] == '--save':
        output_file = "/tmp/vehicle_simulation.gif"
    
    create_simulation(data, output_file=output_file, show=True)


if __name__ == "__main__":
    main()

