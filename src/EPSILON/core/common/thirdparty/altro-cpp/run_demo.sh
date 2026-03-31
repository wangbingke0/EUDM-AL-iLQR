#!/bin/bash
# ALTRO C++ Trajectory Optimization Demo Runner

echo "========================================"
echo "ALTRO C++ Demo - Complete Workflow"
echo "========================================"
echo

# Step 1: Run C++ solver
echo "Step 1/2: Running trajectory optimization solver..."
echo "----------------------------------------"
cd /home/wbk/altro-cpp/build
./bin/unicycle_demo_viz

if [ $? -ne 0 ]; then
    echo "Error: C++ program failed!"
    exit 1
fi

echo
echo "Step 2/2: Launching visualization..."
echo "----------------------------------------"
cd /home/wbk/altro-cpp

# Check if --save flag is provided
if [ "$1" == "--save" ]; then
    echo "Generating image files..."
    python3 visualize_trajectory.py --save
    echo
    echo "Images saved! View with:"
    echo "  display /tmp/unicycle_trajectory.png"
    echo "  display /tmp/unicycle_controls.png"  
    echo "  display /tmp/unicycle_animation.gif"
else
    echo "Launching GUI windows..."
    python3 visualize_trajectory.py
fi

echo
echo "========================================"
echo "Demo Complete!"
echo "========================================"

