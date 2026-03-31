# ALTRO C++ Trajectory Optimization Demo Guide

## Quick Start

### Option 1: One-Command Demo (Recommended)

```bash
# Run with GUI visualization
./run_demo.sh

# Or save images to files
./run_demo.sh --save
```

### Option 2: Manual Step-by-Step

#### Step 1: Run trajectory optimization
```bash
cd build
./bin/unicycle_demo_viz
```

#### Step 2: Visualize results
```bash
# Show interactive GUI windows
python3 visualize_trajectory.py

# Or save to image files
python3 visualize_trajectory.py --save
```

## What You'll See

### Problem Description
- **Robot**: Unicycle model (differential drive robot)
- **Task**: Navigate from (0,0) to (3,3) while avoiding 3 circular obstacles
- **Constraints**: 
  - Control bounds on velocity and angular velocity
  - Obstacle avoidance constraints
  - Terminal goal constraint

### Visualization Windows

#### 1. Trajectory Comparison (Figure 1)
- **Left panel**: Initial trajectory (constant control rollout)
  - Shows where the robot would go with constant initial controls
  - Usually doesn't reach the goal or avoids obstacles
- **Right panel**: Optimized trajectory
  - Successfully navigates to the goal
  - Avoids all obstacles
  - Minimizes cost function

#### 2. Control Inputs (Figure 2)
- **Top plot**: Linear velocity v(t)
- **Bottom plot**: Angular velocity ω(t)
- Shows how the optimized controller varies speed to navigate

#### 3. Animation (Figure 3)
- Real-time playback of the unicycle following the optimized trajectory
- Green rectangle: vehicle
- Blue line: trajectory path
- Red circles: obstacles
- Info box: current state (position, angle, velocities)

## Algorithm Details

### Solver: Augmented Lagrangian iLQR (AL-iLQR)
- **Algorithm**: Iterative Linear Quadratic Regulator with Augmented Lagrangian
- **Integration**: 4th-order Runge-Kutta (RK4)
- **Horizon**: 100 time steps over 5 seconds
- **Convergence**: ~50-60 iterations, ~1.2 seconds solve time

### Key Parameters
- **Penalty parameter ρ**: Starts at 10.0, increases geometrically
- **Regularization**: Levenberg-Marquardt style regularization for numerical stability
- **Tolerances**: 
  - Cost: 1e-4
  - Gradient: 1e-3
  - Constraints: 1e-3

## Output Files

When using `--save` mode:
- `/tmp/unicycle_trajectory.png` - Static trajectory comparison (PNG)
- `/tmp/unicycle_controls.png` - Control input plots (PNG)
- `/tmp/unicycle_animation.gif` - Animated trajectory (GIF)

Raw data files (always generated):
- `/tmp/unicycle_initial.csv` - Initial trajectory data
- `/tmp/unicycle_final.csv` - Optimized trajectory data

## Understanding the Results

### Initial Trajectory
- Uses constant control inputs: v=1.0 m/s, ω=0.5 rad/s
- Results in a curved path due to constant turning
- Does NOT avoid obstacles or reach goal
- This is just a naive guess

### Optimized Trajectory  
- AL-iLQR finds optimal time-varying controls
- Smoothly navigates around all 3 obstacles
- Reaches goal position (3,3) with correct orientation
- Minimizes quadratic cost on states and controls

### Why Initial ≠ Goal?
The initial trajectory is deliberately bad to demonstrate the optimizer's ability to:
1. Find feasible solutions from poor initial guesses
2. Handle multiple conflicting constraints
3. Navigate complex obstacle environments

## Troubleshooting

### "Trajectory files not found"
→ Run the C++ program first: `./build/bin/unicycle_demo_viz`

### "No GUI display"
→ Use save mode: `python3 visualize_trajectory.py --save`
→ Then view images: `display /tmp/unicycle_trajectory.png`

### Matplotlib backend issues
The script automatically tries multiple backends:
- TkAgg (most compatible)
- Qt5Agg (requires PyQt5)
- GTK3Agg (requires GTK)
- WXAgg (requires wxPython)

If none work, use `--save` mode to generate image files.

## Advanced Usage

### Run with different scenarios
Edit `examples/unicycle_demo_viz.cpp` line 67:
```cpp
// Change scenario:
prob_def.SetScenario(altro::problems::UnicycleProblem::kTurn90);
// or
prob_def.SetScenario(altro::problems::UnicycleProblem::kThreeObstacles);
```

### Modify initial guess
Edit `examples/problems/unicycle.cpp` line 36:
```cpp
u0 << 1.0, 0.5;  // [linear_vel, angular_vel]
```

### Change solver parameters
Edit `examples/unicycle_demo_viz.cpp` lines 92-95:
```cpp
solver.SetPenalty(10.0);  // Initial penalty
solver.GetOptions().cost_tolerance = 1e-4;
solver.GetOptions().gradient_tolerance = 1e-3;
solver.GetOptions().constraint_tolerance = 1e-3;
```

Then recompile:
```bash
cd build && make unicycle_demo_viz -j4
```

## References

- [ALTRO Tutorial](https://bjack205.github.io/papers/AL_iLQR_Tutorial.pdf)
- [ALTRO Paper](https://roboticexplorationlab.org/papers/altro-iros.pdf)
- [Project Repository](https://github.com/optimusride/altro-cpp)

