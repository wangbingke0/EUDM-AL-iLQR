# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3;/root/epsilon/src/EPSILON/core/common/thirdparty/altro-cpp;/root/epsilon/src/EPSILON/core/common/thirdparty".split(';') if "${prefix}/include;/usr/include/eigen3;/root/epsilon/src/EPSILON/core/common/thirdparty/altro-cpp;/root/epsilon/src/EPSILON/core/common/thirdparty" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;geometry_msgs;visualization_msgs;tf;common;vehicle_msgs;semantic_map_manager;ssc_planner".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lilqr_planner_lib".split(';') if "-lilqr_planner_lib" != "" else []
PROJECT_NAME = "ilqr_planner"
PROJECT_SPACE_DIR = "/root/epsilon/install"
PROJECT_VERSION = "0.1.0"
