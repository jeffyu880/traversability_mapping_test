cmake_minimum_required(VERSION 3.8)
project(traversability_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(elevation_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

# include directories
include_directories(
  include 
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

# link directories
link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  elevation_msgs
  interactive_markers
  visualization_msgs
  pluginlib
  nav2_core
)

# Range Image Filter
add_executable(traversability_filter src/traversability_filter.cpp)
ament_target_dependencies(traversability_filter ${dependencies})
target_link_libraries(traversability_filter ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping
add_executable(traversability_map src/traversability_map.cpp)
ament_target_dependencies(traversability_map ${dependencies})
target_link_libraries(traversability_map ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# PRM Planner
add_executable(traversability_prm src/traversability_prm.cpp src/library/kdtree.c)
ament_target_dependencies(traversability_prm ${dependencies})
target_link_libraries(traversability_prm ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Path Planner
add_executable(traversability_path src/traversability_path.cpp)
ament_target_dependencies(traversability_path ${dependencies})
target_link_libraries(traversability_path ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# move_base Planner Plugin
add_library(tm_planner_lib SHARED src/library/tm_planner.cpp)
ament_target_dependencies(tm_planner_lib ${dependencies})
target_compile_definitions(tm_planner_lib PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS
  traversability_filter
  traversability_map
  traversability_prm
  traversability_path
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tm_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(tm_planner_lib)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()