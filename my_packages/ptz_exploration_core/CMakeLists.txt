cmake_minimum_required(VERSION 3.16)
project(ptz_exploration_core)

# Default to an optimized build when no explicit build type is provided.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# Default to C++20 for projector/optimizer targets.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Runtime-oriented flags for optimized builds.
if(CMAKE_BUILD_TYPE MATCHES "Release|RelWithDebInfo")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3)
  endif()

  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
  if(ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(STATUS "IPO/LTO not supported: ${ipo_error}")
  endif()
endif()

# GCC 10+ may emit -Wpsabi notes from Boost internals (chi-squared quantile path).
# This is non-fatal and does not indicate a runtime issue in this package.
if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wno-psabi)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(DIRECTORY ptz_exploration_core/
  DESTINATION ${PYTHON_INSTALL_DIR}/ptz_exploration_core
  PATTERN "__pycache__" EXCLUDE
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Landmark.msg"
  "msg/LandmarkArray.msg"
  "msg/DetectionMeasurement.msg"
  "msg/DetectionMeasurementArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

add_library(utils_lib
  src/utils/math.cpp
  src/utils/ros.cpp
  src/utils/data_association.cpp
)

target_link_libraries(utils_lib Eigen3::Eigen)
if(TARGET gtsam)
  target_link_libraries(utils_lib gtsam)
elseif(TARGET gtsam::gtsam)
  target_link_libraries(utils_lib gtsam::gtsam)
else()
  target_link_libraries(utils_lib ${GTSAM_LIBRARIES})
endif()

ament_target_dependencies(utils_lib
  rclcpp
  geometry_msgs
  octomap
  octomap_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  vision_msgs
)

add_executable(projector
  src/projector.cpp
)
target_link_libraries(projector utils_lib ${cpp_typesupport_target})
ament_target_dependencies(projector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  visualization_msgs
  octomap_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  octomap
  image_geometry
  cv_bridge
)

add_executable(optimizer
  src/optimizer.cpp
)
target_link_libraries(optimizer utils_lib ${cpp_typesupport_target})
if(TARGET gtsam)
  target_link_libraries(optimizer gtsam)
elseif(TARGET gtsam::gtsam)
  target_link_libraries(optimizer gtsam::gtsam)
else()
  target_link_libraries(optimizer ${GTSAM_LIBRARIES})
endif()
ament_target_dependencies(optimizer
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

add_executable(gazer
  src/gazer.cpp
)
target_link_libraries(gazer utils_lib ${cpp_typesupport_target})
ament_target_dependencies(gazer
  rclcpp
  geometry_msgs
  nav_msgs
  octomap
  octomap_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(cloud_in
  src/cloud_in.cpp
)
target_link_libraries(cloud_in utils_lib)
ament_target_dependencies(cloud_in
  rclcpp
  sensor_msgs
  std_msgs
)

install(PROGRAMS
  ptz_exploration_core/detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detector
)

install(PROGRAMS
  ptz_exploration_core/explorer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME explorer
)

install(PROGRAMS
  ptz_exploration_core/collector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collector
)

install(PROGRAMS
  ptz_exploration_core/plotter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plotter
)

install(PROGRAMS
  ptz_exploration_core/plot_factor_graph.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_factor_graph
)

install(TARGETS
  projector
  optimizer
  gazer
  cloud_in
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
