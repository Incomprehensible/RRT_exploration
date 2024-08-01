cmake_minimum_required(VERSION 3.8)
project(single_agent_rrt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common search kdtree segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

# Local Frontier Detector

add_library(local_frontier_detector SHARED
  src/local_frontier_detector.cpp)

target_include_directories(local_frontier_detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(local_frontier_detector
    PRIVATE "SINGLE_AGENT_RRT_BUILDING_DLL")

ament_target_dependencies(local_frontier_detector
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "rcl_interfaces"
    "std_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
    "nav2_msgs"
    "nav_msgs"
    "nav2_util"
    "visualization_msgs"
  )

rclcpp_components_register_node(local_frontier_detector PLUGIN "LocalFrontierDetector" EXECUTABLE local_frontier_detector_node)

# Filter

add_library(filter SHARED
  src/filter.cpp)

target_include_directories(filter PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(filter ${PCL_LIBRARIES})

target_compile_definitions(filter
    PRIVATE "SINGLE_AGENT_RRT_BUILDING_DLL")

ament_target_dependencies(filter
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "rcl_interfaces"
    "std_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
    "nav2_msgs"
    "nav_msgs"
    "nav2_util"
    "visualization_msgs"
  )

rclcpp_components_register_node(filter PLUGIN "Filter" EXECUTABLE filter_node)

install(TARGETS
    local_frontier_detector
    filter
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

add_executable(rrt_exploration
  src/main.cpp)
target_link_libraries(rrt_exploration
    local_frontier_detector
    filter
)
ament_target_dependencies(rrt_exploration
  "rclcpp"
)

install(TARGETS
    rrt_exploration
    DESTINATION lib/${PROJECT_NAME}
)

# add_executable(robot_task_allocator src/robot_task_allocator.cpp)
# ament_target_dependencies(frontier_filter rclcpp geometry_msgs sensor_msgs nav2_msgs tf2_geometry_msgs rclcpp_action visualization_msgs tf2_ros)

# add_executable(local_frontier_detector src/local_frontier_detector.cpp)
# ament_target_dependencies(local_frontier_detector rclcpp geometry_msgs sensor_msgs nav2_msgs tf2_geometry_msgs rclcpp_action visualization_msgs tf2_ros)

# add_executable(frontier_filter src/frontier_filter.cpp)
# target_link_libraries(frontier_filter ${PCL_LIBRARIES})
# ament_target_dependencies(frontier_filter rclcpp geometry_msgs sensor_msgs nav2_msgs tf2_geometry_msgs rclcpp_action visualization_msgs tf2_ros)

# install header files
# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# install(TARGETS
#     # robot_task_allocator
#     local_frontier_detector
#     # frontier_filter
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
