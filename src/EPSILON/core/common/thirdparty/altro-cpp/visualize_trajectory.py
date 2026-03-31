#!/usr/bin/env python3
"""
ALTRO C++ Trajectory Optimization Visualization
Visualize unicycle obstacle avoidance trajectory optimization results
"""

import sys
import numpy as np

# Force GUI backend if not in save mode
if '--save' not in sys.argv:
    import matplotlib
    # Try different backends in order of preference
    for backend in ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'WXAgg']:
        try:
            matplotlib.use(backend)
            print(f"Using matplotlib backend: {backend}")
            break
        except:
            continue

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

def load_trajectory(filename):
    """Load trajectory data"""
    states = []
    obstacles = []
    goal = None
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            if line.startswith('OBS'):
                parts = line.split(',')
                obstacles.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith('GOAL'):
                parts = line.split(',')
                goal = [float(parts[1]), float(parts[2]), float(parts[3])]
            else:
                parts = line.split(',')
                states.append([float(x) for x in parts])
    
    return np.array(states), obstacles, goal

def plot_static_trajectory(states_init, states_final, obstacles, goal, save_fig=False):
    """Plot static trajectory comparison"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # Initial trajectory
    ax1.set_title('Initial Trajectory', fontsize=14, fontweight='bold')
    plot_trajectory_on_ax(ax1, states_init, obstacles, goal, 'Initial')
    
    # Optimized trajectory
    ax2.set_title('Optimized Trajectory', fontsize=14, fontweight='bold')
    plot_trajectory_on_ax(ax2, states_final, obstacles, goal, 'Optimized')
    
    plt.tight_layout()
    if save_fig:
        plt.savefig('/tmp/unicycle_trajectory.png', dpi=150, bbox_inches='tight')
        print("✓ Static trajectory plot saved: /tmp/unicycle_trajectory.png")
    return fig

def plot_trajectory_on_ax(ax, states, obstacles, goal, label):
    """Plot trajectory on specified axes"""
    x = states[:, 0]
    y = states[:, 1]
    theta = states[:, 2]
    
    # Plot trajectory
    ax.plot(x, y, 'b-', linewidth=2, label='Trajectory', zorder=3)
    ax.plot(x[0], y[0], 'go', markersize=12, label='Start', zorder=5)
    ax.plot(x[-1], y[-1], 'r*', markersize=15, label='End', zorder=5)
    
    # Plot obstacles
    for obs in obstacles:
        circle = Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.3, zorder=2)
        ax.add_patch(circle)
        ax.plot(obs[0], obs[1], 'rx', markersize=10, zorder=4)
    
    # Plot goal position
    if goal:
        ax.plot(goal[0], goal[1], 'r*', markersize=15, zorder=5)
        # Plot goal direction
        arrow_len = 0.3
        dx = arrow_len * np.cos(goal[2])
        dy = arrow_len * np.sin(goal[2])
        ax.arrow(goal[0], goal[1], dx, dy, head_width=0.15, 
                head_length=0.1, fc='red', ec='red', zorder=5)
    
    # Draw vehicle orientation at intervals
    step = max(1, len(states) // 20)
    for i in range(0, len(states), step):
        arrow_len = 0.2
        dx = arrow_len * np.cos(theta[i])
        dy = arrow_len * np.sin(theta[i])
        ax.arrow(x[i], y[i], dx, dy, head_width=0.1, 
                head_length=0.08, fc='blue', ec='blue', alpha=0.6, zorder=4)
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=10)
    
    # Set axis limits
    margin = 0.5
    ax.set_xlim(min(x.min(), 0) - margin, max(x.max(), goal[0] if goal else 0) + margin)
    ax.set_ylim(min(y.min(), 0) - margin, max(y.max(), goal[1] if goal else 0) + margin)

def create_animation(states, obstacles, goal, save_anim=False, filename='/tmp/unicycle_animation.gif'):
    """Create animation"""
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title('ALTRO C++ - Unicycle Obstacle Avoidance Trajectory Animation', fontsize=14, fontweight='bold')
    
    x = states[:, 0]
    y = states[:, 1]
    theta = states[:, 2]
    
    # Draw obstacles
    for obs in obstacles:
        circle = Circle((obs[0], obs[1]), obs[2], color='red', alpha=0.3, zorder=2)
        ax.add_patch(circle)
        ax.plot(obs[0], obs[1], 'rx', markersize=10, zorder=4)
    
    # Draw goal
    if goal:
        ax.plot(goal[0], goal[1], 'r*', markersize=15, label='Goal', zorder=5)
        arrow_len = 0.3
        dx = arrow_len * np.cos(goal[2])
        dy = arrow_len * np.sin(goal[2])
        ax.arrow(goal[0], goal[1], dx, dy, head_width=0.15, 
                head_length=0.1, fc='red', ec='red', zorder=5)
    
    # Initialize plot elements
    line, = ax.plot([], [], 'b-', linewidth=2, label='Trajectory', zorder=3)
    point, = ax.plot([], [], 'go', markersize=10, label='Current Position', zorder=5)
    
    # Vehicle direction arrow
    vehicle_arrow = FancyArrowPatch((0, 0), (0, 0), 
                                   arrowstyle='->', mutation_scale=20, 
                                   lw=2, color='green', zorder=6)
    ax.add_patch(vehicle_arrow)
    
    # Vehicle outline (simplified as rectangle)
    vehicle_width = 0.15
    vehicle_length = 0.25
    vehicle_rect = mpatches.Rectangle((0, 0), vehicle_length, vehicle_width,
                                     fc='green', ec='darkgreen', alpha=0.7, zorder=6)
    ax.add_patch(vehicle_rect)
    
    # Text info
    info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                       verticalalignment='top', fontsize=10,
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left', fontsize=10)
    
    # Set axis limits
    margin = 0.5
    ax.set_xlim(min(x.min(), 0) - margin, max(x.max(), goal[0] if goal else 0) + margin)
    ax.set_ylim(min(y.min(), 0) - margin, max(y.max(), goal[1] if goal else 0) + margin)
    
    def init():
        line.set_data([], [])
        point.set_data([], [])
        vehicle_arrow.set_positions((0, 0), (0, 0))
        vehicle_rect.set_xy((0, 0))
        info_text.set_text('')
        return line, point, vehicle_arrow, vehicle_rect, info_text
    
    def animate(i):
        # Update trajectory line
        line.set_data(x[:i+1], y[:i+1])
        
        # Update current position
        point.set_data([x[i]], [y[i]])
        
        # Update vehicle direction arrow
        arrow_len = 0.3
        dx = arrow_len * np.cos(theta[i])
        dy = arrow_len * np.sin(theta[i])
        vehicle_arrow.set_positions((x[i], y[i]), (x[i] + dx, y[i] + dy))
        
        # Update vehicle rectangle (rotated)
        rect_x = x[i] - vehicle_length/2 * np.cos(theta[i]) + vehicle_width/2 * np.sin(theta[i])
        rect_y = y[i] - vehicle_length/2 * np.sin(theta[i]) - vehicle_width/2 * np.cos(theta[i])
        
        # Create rotated rectangle
        import matplotlib.transforms as transforms
        t = transforms.Affine2D().rotate_around(x[i], y[i], theta[i]) + ax.transData
        vehicle_rect.set_transform(t)
        vehicle_rect.set_xy((rect_x, rect_y))
        
        # Update info text
        v = states[i, 3] if i < len(states) - 1 else 0
        omega = states[i, 4] if i < len(states) - 1 else 0
        info_text.set_text(f'Step: {i}/{len(states)-1}\n'
                          f'Position: ({x[i]:.2f}, {y[i]:.2f})\n'
                          f'Angle: {np.degrees(theta[i]):.1f}°\n'
                          f'Velocity: {v:.2f} m/s\n'
                          f'Angular vel: {omega:.2f} rad/s')
        
        return line, point, vehicle_arrow, vehicle_rect, info_text
    
    # Create animation
    frames = len(states)
    interval = 50  # milliseconds
    
    anim = FuncAnimation(fig, animate, init_func=init, frames=frames,
                        interval=interval, blit=True, repeat=True)
    
    # Save animation if requested
    if save_anim:
        print(f"Generating animation... ({frames} frames)")
        anim.save(filename, writer='pillow', fps=20, dpi=100)
        print(f"✓ Animation saved: {filename}")
    
    return fig, anim

def plot_controls(states, save_fig=False, filename='/tmp/unicycle_controls.png'):
    """Plot control inputs"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    time_steps = np.arange(len(states))
    v = states[:, 3]  # linear velocity
    omega = states[:, 4]  # angular velocity
    
    # Linear velocity
    ax1.plot(time_steps, v, 'b-', linewidth=2, label='Linear velocity v')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_xlabel('Time Step', fontsize=12)
    ax1.set_ylabel('Linear Velocity (m/s)', fontsize=12)
    ax1.set_title('Control Input - Linear Velocity', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)
    
    # Angular velocity
    ax2.plot(time_steps, omega, 'r-', linewidth=2, label='Angular velocity ω')
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time Step', fontsize=12)
    ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    ax2.set_title('Control Input - Angular Velocity', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)
    
    plt.tight_layout()
    if save_fig:
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"✓ Control input plot saved: {filename}")
    return fig

def main():
    import sys
    
    # Check if save mode is requested
    save_mode = '--save' in sys.argv
    
    print("=" * 60)
    print("ALTRO C++ Trajectory Optimization Visualization")
    print("=" * 60)
    print()
    
    if save_mode:
        print("Running in SAVE mode - generating image files...")
    else:
        print("Running in GUI mode - displaying interactive windows...")
    print()
    
    # Load data
    print("Loading trajectory data...")
    try:
        states_init, obstacles, goal = load_trajectory('/tmp/unicycle_initial.csv')
        states_final, _, _ = load_trajectory('/tmp/unicycle_final.csv')
        print(f"✓ Initial trajectory: {len(states_init)} points")
        print(f"✓ Optimized trajectory: {len(states_final)} points")
        print(f"✓ Number of obstacles: {len(obstacles)}")
        print()
    except FileNotFoundError:
        print("Error: Trajectory files not found. Please run the C++ program first.")
        return
    
    # 1. Plot static comparison
    print("Generating static trajectory comparison plot...")
    fig1 = plot_static_trajectory(states_init, states_final, obstacles, goal, save_fig=save_mode)
    
    # 2. Plot control inputs
    print("Generating control input plot...")
    fig2 = plot_controls(states_final, save_fig=save_mode)
    
    # 3. Create animation
    print("Generating trajectory animation...")
    fig3, anim = create_animation(states_final, obstacles, goal, save_anim=save_mode)
    
    print()
    print("=" * 60)
    print("Visualization Complete!")
    print("=" * 60)
    
    if save_mode:
        print("Generated files:")
        print("  - /tmp/unicycle_trajectory.png    (Static trajectory comparison)")
        print("  - /tmp/unicycle_controls.png      (Control inputs)")
        print("  - /tmp/unicycle_animation.gif     (Animation)")
        print()
        print("To view images:")
        print("  display /tmp/unicycle_trajectory.png")
        print("  display /tmp/unicycle_controls.png")
        print("  display /tmp/unicycle_animation.gif")
    else:
        print("Displaying interactive windows...")
        print("Close windows to exit.")
        print()
        print("Tip: Run with --save flag to save images instead:")
        print("  python3 visualize_trajectory.py --save")
        
        # Show interactive GUI
        plt.show()

if __name__ == '__main__':
    main()

