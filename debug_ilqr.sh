#!/bin/bash
# GDB debugging script for iLQR planner

# Set up ROS environment
source /opt/ros/melodic/setup.bash
source /root/epsilon/devel/setup.bash

# Find the executable
EXECUTABLE="/root/epsilon/devel/lib/planning_integrated/test_ilqr_with_eudm"

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable not found at $EXECUTABLE"
    echo "Please compile the project first with: catkin_make"
    exit 1
fi

echo "Starting GDB debugging session..."
echo "Executable: $EXECUTABLE"
echo ""
echo "Useful GDB commands:"
echo "  (gdb) break LQRWarmStart          # Set breakpoint at LQR warm start"
echo "  (gdb) break ExtendReferenceTrajectoryToMinimumLength  # Set breakpoint at trajectory extension"
echo "  (gdb) break VisualizeLQRWarmStartTrajectory  # Set breakpoint at visualization"
echo "  (gdb) run                          # Start the program"
echo "  (gdb) bt                           # Show backtrace when crashed"
echo "  (gdb) print traj                    # Print trajectory object"
echo "  (gdb) print N                       # Print N value"
echo "  (gdb) print lqr_warm_start_traj_   # Print saved trajectory"
echo "  (gdb) continue                      # Continue execution"
echo "  (gdb) next                          # Step to next line"
echo "  (gdb) step                          # Step into function"
echo ""

# Start GDB
gdb -ex "set args ~arena_info_static:=/arena_info_static ~arena_info_dynamic:=/arena_info_dynamic ~ctrl:=/ctrl/agent_0 __name:=test_ilqr_with_eudm_0" \
    -ex "set environment ROS_MASTER_URI=http://localhost:11311" \
    -ex "handle SIGSEGV stop print" \
    -ex "handle SIGABRT stop print" \
    "$EXECUTABLE"

