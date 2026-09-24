cmake_minimum_required(VERSION 3.8)
project(cloud_pipeline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

# OptiTrack bridge messages (mocap2ros2/optitrack_packages_ros2).
# Source that workspace's install/setup.bash before building this one.
find_package(optitrack_multiplexer_ros2_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters features search io segmentation surface)

add_executable(map_pip src/map_pip.cpp)
add_executable(map_pip_mocap src/map_pip_mocap.cpp)
add_executable(map_pip_mocap_normals src/map_pip_mocap_normals.cpp)
add_executable(map_pip_dlio_multinormals src/map_pip_dlio_multinormals.cpp)
add_executable(map_pip_dlio_multinormals_test src/map_pip_dlio_multinormals_test.cpp)
add_executable(map_pip_optitrack_slerp src/map_pip_optitrack_slerp.cpp)
add_executable(map_pip_optitrack_tf src/map_pip_optitrack_tf.cpp)
add_executable(mocap_tf_broadcaster src/mocap_tf_broadcaster.cpp)


add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(map_pip
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(map_pip_mocap
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(map_pip_mocap_normals
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  yaml-cpp
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(map_pip_dlio_multinormals
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  yaml-cpp
  std_srvs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(map_pip_dlio_multinormals_test
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  yaml-cpp
  std_srvs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(mocap_tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_ros
  optitrack_multiplexer_ros2_msgs
)

ament_target_dependencies(map_pip_optitrack_slerp
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  yaml-cpp
  std_srvs
  nav_msgs
  optitrack_multiplexer_ros2_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(map_pip_optitrack_tf
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  yaml-cpp
  std_srvs
  nav_msgs
  optitrack_multiplexer_ros2_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)


target_link_libraries(map_pip ${PCL_LIBRARIES})
target_include_directories(map_pip PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(map_pip_mocap_normals
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(map_pip_dlio_multinormals
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

target_include_directories(map_pip_dlio_multinormals PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(map_pip_dlio_multinormals_test
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

target_include_directories(map_pip_dlio_multinormals_test PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(map_pip_optitrack_slerp
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

target_include_directories(map_pip_optitrack_slerp PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(map_pip_optitrack_tf
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

target_include_directories(map_pip_optitrack_tf PRIVATE
  ${PCL_INCLUDE_DIRS}
)

install(
  TARGETS 
  map_pip
  map_pip_mocap
  map_pip_mocap_normals
  map_pip_dlio_multinormals
  map_pip_dlio_multinormals_test
  map_pip_optitrack_slerp
  mocap_tf_broadcaster
  map_pip_optitrack_tf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
