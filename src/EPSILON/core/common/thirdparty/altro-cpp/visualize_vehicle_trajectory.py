#!/usr/bin/env python3
"""
Vehicle Trajectory Visualization Script

Visualizes the trajectory optimization results for the vehicle kinematic model.
State: [x, y, yaw, v, a, kappa]
Control: [da, dkappa]
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle, FancyBboxPatch
from matplotlib.collections import LineCollection
import matplotlib.gridspec as gridspec
import os
import sys

# Set plot style
plt.style.use('seaborn-v0_8-whitegrid')

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

def draw_vehicle(ax, x, y, yaw, length=2.0, width=1.0, color='steelblue', alpha=0.7):
    """Draw a vehicle rectangle at the given position and orientation."""
    # Vehicle corners (centered at rear axle)
    corners = np.array([
        [-length * 0.3, -width / 2],
        [length * 0.7, -width / 2],
        [length * 0.7, width / 2],
        [-length * 0.3, width / 2],
        [-length * 0.3, -width / 2]
    ])
    
    # Rotation matrix
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    
    # Transform corners
    corners_rot = corners @ R.T + np.array([x, y])
    
    ax.fill(corners_rot[:, 0], corners_rot[:, 1], color=color, alpha=alpha, edgecolor='black', linewidth=1)
    
    # Draw heading arrow
    arrow_len = length * 0.5
    ax.arrow(x, y, arrow_len * np.cos(yaw), arrow_len * np.sin(yaw),
             head_width=0.3, head_length=0.15, fc='darkred', ec='darkred', zorder=10)

def plot_trajectory(initial_file, final_file, output_file=None):
    """Plot initial and final trajectories with state and control profiles."""
    
    # Check if files exist
    if not os.path.exists(initial_file):
        print(f"Error: Initial trajectory file not found: {initial_file}")
        return
    if not os.path.exists(final_file):
        print(f"Error: Final trajectory file not found: {final_file}")
        return
    
    # Load data
    initial = load_trajectory(initial_file)
    final = load_trajectory(final_file)
    
    if len(initial['x']) == 0 or len(final['x']) == 0:
        print("Error: Empty trajectory data")
        return
    
    # Create figure with subplots
    fig = plt.figure(figsize=(18, 12))
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # Main trajectory plot (large, spans 2 rows and 2 columns)
    ax_main = fig.add_subplot(gs[0:2, 0:2])
    
    # State plots
    ax_v = fig.add_subplot(gs[0, 2])
    ax_a = fig.add_subplot(gs[1, 2])
    ax_kappa = fig.add_subplot(gs[2, 0])
    
    # Control plots
    ax_da = fig.add_subplot(gs[2, 1])
    ax_dk = fig.add_subplot(gs[2, 2])
    
    # ==================== Main Trajectory Plot ====================
    # Plot initial trajectory (dashed, gray)
    ax_main.plot(initial['x'], initial['y'], '--', color='gray', linewidth=1.5, 
                 label='Initial Trajectory', alpha=0.6)
    
    # Plot final trajectory with velocity coloring
    if len(final['x']) > 1:
        points = np.array([final['x'], final['y']]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        norm = plt.Normalize(final['v'].min(), final['v'].max())
        lc = LineCollection(segments, cmap='viridis', norm=norm, linewidth=3)
        lc.set_array(final['v'][:-1])
        line = ax_main.add_collection(lc)
        cbar = fig.colorbar(line, ax=ax_main, label='Velocity [m/s]', shrink=0.8)
    
    # Draw vehicles along trajectory
    n_vehicles = min(8, len(final['x']))
    indices = np.linspace(0, len(final['x']) - 1, n_vehicles, dtype=int)
    for i, idx in enumerate(indices):
        alpha = 0.3 + 0.7 * (i / (n_vehicles - 1)) if n_vehicles > 1 else 1.0
        draw_vehicle(ax_main, final['x'][idx], final['y'][idx], final['yaw'][idx],
                    length=2.0, width=1.0, color='steelblue', alpha=alpha)
    
    # Draw obstacles (if any)
    for obs in final['obstacles']:
        obs_x, obs_y, obs_r = obs
        # Draw obstacle circle
        circle = plt.Circle((obs_x, obs_y), obs_r, color='red', alpha=0.3, 
                            label='Obstacle' if obs == final['obstacles'][0] else '')
        ax_main.add_patch(circle)
        # Draw obstacle outline
        circle_outline = plt.Circle((obs_x, obs_y), obs_r, color='darkred', 
                                     fill=False, linewidth=2)
        ax_main.add_patch(circle_outline)
        # Draw vehicle icon in obstacle
        draw_vehicle(ax_main, obs_x, obs_y, 0, length=2.0, width=1.0, 
                    color='darkred', alpha=0.8)
    
    # Mark start and goal
    ax_main.plot(initial['x'][0], initial['y'][0], 'go', markersize=15, 
                 label='Start', zorder=20)
    if final['goal'] is not None:
        ax_main.plot(final['goal'][0], final['goal'][1], 'r*', markersize=20, 
                     label='Goal', zorder=20)
    ax_main.plot(final['x'][-1], final['y'][-1], 'bs', markersize=12, 
                 label='Final Position', zorder=20)
    
    ax_main.set_xlabel('X Position [m]', fontsize=12)
    ax_main.set_ylabel('Y Position [m]', fontsize=12)
    ax_main.set_title('Vehicle Trajectory Optimization Result', fontsize=14, fontweight='bold')
    ax_main.legend(loc='upper left', fontsize=10)
    ax_main.set_aspect('equal')
    ax_main.grid(True, alpha=0.3)
    
    # ==================== State Plots ====================
    # Velocity
    ax_v.plot(initial['t'], initial['v'], '--', color='gray', linewidth=1.5, 
              label='Initial', alpha=0.6)
    ax_v.plot(final['t'], final['v'], '-', color='tab:blue', linewidth=2, label='Optimized')
    ax_v.set_xlabel('Time [s]')
    ax_v.set_ylabel('Velocity [m/s]')
    ax_v.set_title('Velocity Profile')
    ax_v.legend(fontsize=9)
    ax_v.grid(True, alpha=0.3)
    
    # Acceleration
    ax_a.plot(initial['t'], initial['a'], '--', color='gray', linewidth=1.5, 
              label='Initial', alpha=0.6)
    ax_a.plot(final['t'], final['a'], '-', color='tab:orange', linewidth=2, label='Optimized')
    ax_a.set_xlabel('Time [s]')
    ax_a.set_ylabel('Acceleration [m/s²]')
    ax_a.set_title('Acceleration Profile')
    ax_a.legend(fontsize=9)
    ax_a.grid(True, alpha=0.3)
    
    # Curvature
    ax_kappa.plot(initial['t'], initial['kappa'], '--', color='gray', linewidth=1.5, 
                  label='Initial', alpha=0.6)
    ax_kappa.plot(final['t'], final['kappa'], '-', color='tab:green', linewidth=2, label='Optimized')
    ax_kappa.set_xlabel('Time [s]')
    ax_kappa.set_ylabel('Curvature [1/m]')
    ax_kappa.set_title('Curvature Profile')
    ax_kappa.legend(fontsize=9)
    ax_kappa.grid(True, alpha=0.3)
    
    # ==================== Control Plots ====================
    # Jerk (da)
    ax_da.plot(initial['t'][:-1], initial['da'][:-1], '--', color='gray', linewidth=1.5, 
               label='Initial', alpha=0.6)
    ax_da.plot(final['t'][:-1], final['da'][:-1], '-', color='tab:red', linewidth=2, label='Optimized')
    ax_da.set_xlabel('Time [s]')
    ax_da.set_ylabel('Jerk [m/s³]')
    ax_da.set_title('Control: Jerk (da)')
    ax_da.legend(fontsize=9)
    ax_da.grid(True, alpha=0.3)
    
    # Curvature rate (dkappa)
    ax_dk.plot(initial['t'][:-1], initial['dkappa'][:-1], '--', color='gray', linewidth=1.5, 
               label='Initial', alpha=0.6)
    ax_dk.plot(final['t'][:-1], final['dkappa'][:-1], '-', color='tab:purple', linewidth=2, label='Optimized')
    ax_dk.set_xlabel('Time [s]')
    ax_dk.set_ylabel('Curvature Rate [1/m/s]')
    ax_dk.set_title('Control: Curvature Rate (dκ)')
    ax_dk.legend(fontsize=9)
    ax_dk.grid(True, alpha=0.3)
    
    # Add overall title
    fig.suptitle('Vehicle Trajectory Optimization - ALTRO Solver', fontsize=16, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {output_file}")
    
    plt.show()

def plot_animation(final_file, output_gif=None):
    """Create an animation of the vehicle moving along the trajectory."""
    from matplotlib.animation import FuncAnimation
    
    if not os.path.exists(final_file):
        print(f"Error: Trajectory file not found: {final_file}")
        return
    
    data = load_trajectory(final_file)
    
    if len(data['x']) == 0:
        print("Error: Empty trajectory data")
        return
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Set axis limits
    margin = 5
    ax.set_xlim(min(data['x']) - margin, max(data['x']) + margin)
    ax.set_ylim(min(data['y']) - margin, max(data['y']) + margin)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.set_title('Vehicle Trajectory Animation')
    
    # Plot full trajectory
    ax.plot(data['x'], data['y'], 'b-', alpha=0.3, linewidth=1)
    
    # Mark start and goal
    ax.plot(data['x'][0], data['y'][0], 'go', markersize=12, label='Start')
    if data['goal'] is not None:
        ax.plot(data['goal'][0], data['goal'][1], 'r*', markersize=15, label='Goal')
    ax.legend()
    
    # Vehicle patch (will be updated in animation)
    vehicle_patch = plt.Polygon(np.zeros((5, 2)), closed=True, 
                                 facecolor='steelblue', edgecolor='black', linewidth=1.5)
    ax.add_patch(vehicle_patch)
    
    # Trail line
    trail, = ax.plot([], [], 'b-', linewidth=2, alpha=0.7)
    
    # Time text
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                        fontsize=12, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    def init():
        vehicle_patch.set_xy(np.zeros((5, 2)))
        trail.set_data([], [])
        time_text.set_text('')
        return vehicle_patch, trail, time_text
    
    def update(frame):
        # Vehicle corners
        length, width = 2.0, 1.0
        corners = np.array([
            [-length * 0.3, -width / 2],
            [length * 0.7, -width / 2],
            [length * 0.7, width / 2],
            [-length * 0.3, width / 2],
            [-length * 0.3, -width / 2]
        ])
        
        # Rotation
        yaw = data['yaw'][frame]
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        corners_rot = corners @ R.T + np.array([data['x'][frame], data['y'][frame]])
        
        vehicle_patch.set_xy(corners_rot)
        
        # Update trail
        trail.set_data(data['x'][:frame+1], data['y'][:frame+1])
        
        # Update time text
        time_text.set_text(f't = {data["t"][frame]:.2f}s\nv = {data["v"][frame]:.2f} m/s')
        
        return vehicle_patch, trail, time_text
    
    anim = FuncAnimation(fig, update, frames=len(data['x']),
                         init_func=init, blit=True, interval=50)
    
    if output_gif:
        anim.save(output_gif, writer='pillow', fps=20)
        print(f"Animation saved to: {output_gif}")
    
    plt.show()

def main():
    initial_file = "/tmp/vehicle_initial.csv"
    final_file = "/tmp/vehicle_final.csv"
    
    # Check command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == '--animate':
        plot_animation(final_file, output_gif="/tmp/vehicle_trajectory.gif")
    else:
        plot_trajectory(initial_file, final_file, output_file="/tmp/vehicle_trajectory.png")

if __name__ == "__main__":
    main()

