cmake_minimum_required(VERSION 3.10)
project(ilqr_planner)

# C++14 is required for altro-cpp
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Release mode: optimize for performance
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  common
  vehicle_msgs
  semantic_map_manager
  ssc_planner
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# Get absolute path for thirdparty directory
get_filename_component(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../core/common/thirdparty" ABSOLUTE)

# altro-cpp paths (now in thirdparty)
set(ALTRO_CPP_DIR ${THIRDPARTY_DIR}/altro-cpp)
set(ALTRO_CPP_BUILD_DIR ${ALTRO_CPP_DIR}/build)

catkin_package(
  INCLUDE_DIRS inc ${EIGEN3_INCLUDE_DIRS} ${ALTRO_CPP_DIR} ${THIRDPARTY_DIR}
  LIBRARIES ilqr_planner_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs tf common vehicle_msgs semantic_map_manager ssc_planner
)

include_directories(
  inc
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${THIRDPARTY_DIR}
  # altro-cpp includes
  ${ALTRO_CPP_DIR}
  ${ALTRO_CPP_DIR}/altro
  ${ALTRO_CPP_DIR}/examples
)

# altro-cpp source files (compile directly instead of linking prebuilt)
set(ALTRO_SOURCES
  ${ALTRO_CPP_DIR}/altro/common/functionbase.cpp
  ${ALTRO_CPP_DIR}/altro/common/knotpoint.cpp
  ${ALTRO_CPP_DIR}/altro/common/trajectory.cpp
  ${ALTRO_CPP_DIR}/altro/common/solver_options.cpp
  ${ALTRO_CPP_DIR}/altro/common/solver_stats.cpp
  ${ALTRO_CPP_DIR}/altro/common/solver_logger.cpp
  ${ALTRO_CPP_DIR}/altro/common/log_entry.cpp
  ${ALTRO_CPP_DIR}/altro/common/profile_entry.cpp
  ${ALTRO_CPP_DIR}/altro/common/timer.cpp
  ${ALTRO_CPP_DIR}/altro/common/threadpool.cpp
  ${ALTRO_CPP_DIR}/altro/common/threadsafe_queue.cpp
  ${ALTRO_CPP_DIR}/altro/utils/assert.cpp
  ${ALTRO_CPP_DIR}/altro/utils/derivative_checker.cpp
  ${ALTRO_CPP_DIR}/altro/problem/costfunction.cpp
  ${ALTRO_CPP_DIR}/altro/problem/dynamics.cpp
  ${ALTRO_CPP_DIR}/altro/problem/integration.cpp
  ${ALTRO_CPP_DIR}/altro/problem/discretized_model.cpp
  ${ALTRO_CPP_DIR}/altro/problem/problem.cpp
  ${ALTRO_CPP_DIR}/altro/constraints/constraint.cpp
  ${ALTRO_CPP_DIR}/altro/constraints/constraint_values.cpp
  ${ALTRO_CPP_DIR}/altro/ilqr/ilqr.cpp
  ${ALTRO_CPP_DIR}/altro/ilqr/cost_expansion.cpp
  ${ALTRO_CPP_DIR}/altro/ilqr/dynamics_expansion.cpp
  ${ALTRO_CPP_DIR}/altro/ilqr/knot_point_function_type.cpp
  ${ALTRO_CPP_DIR}/altro/augmented_lagrangian/al_cost.cpp
  ${ALTRO_CPP_DIR}/altro/augmented_lagrangian/al_problem.cpp
  ${ALTRO_CPP_DIR}/altro/augmented_lagrangian/al_solver.cpp
  # Examples we need
  ${ALTRO_CPP_DIR}/examples/quadratic_cost.cpp
  ${ALTRO_CPP_DIR}/examples/basic_constraints.cpp
  ${ALTRO_CPP_DIR}/examples/obstacle_constraints.cpp
)

# iLQR Planner library
add_library(ilqr_planner_lib
  ${ALTRO_SOURCES}
  src/ilqr_planner/cartesian_dynamics.cpp
  src/ilqr_planner/cartesian_corridor_constraint.cpp
  src/ilqr_planner/ilqr_lqr_warm_start.cpp
  src/ilqr_planner/ilqr_al_ilqr_solver.cpp
  src/ilqr_planner/ilqr_planner.cpp
  src/ilqr_planner/ilqr_visualizer.cpp
  src/ilqr_planner/ilqr_server_ros.cpp
)

target_compile_definitions(ilqr_planner_lib PRIVATE
  # Disable fmt features that require newer fmt versions
  FMT_USE_CONSTEXPR=0
)

target_link_libraries(ilqr_planner_lib
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
  pthread
)

target_compile_options(ilqr_planner_lib PRIVATE ${OpenMP_CXX_FLAGS})

add_dependencies(ilqr_planner_lib
  ${catkin_EXPORTED_TARGETS}
)

# Test node
add_executable(ilqr_planner_node
  src/ilqr_planner_node.cpp
)

target_link_libraries(ilqr_planner_node
  ilqr_planner_lib
  ${catkin_LIBRARIES}
)

add_dependencies(ilqr_planner_node
  ${catkin_EXPORTED_TARGETS}
)
