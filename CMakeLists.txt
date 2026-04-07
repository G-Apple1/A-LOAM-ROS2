cmake_minimum_required(VERSION 3.8)
project(aloam_velodyne_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(glog REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# 第三方库
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ==============================================
# A-LOAM 核心节点
# ==============================================

# 1. 特征提取
add_executable(ascanRegistration src/scanRegistration.cpp)
ament_target_dependencies(ascanRegistration
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  Eigen3
)
target_link_libraries(ascanRegistration
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 2. 激光里程计
add_executable(alaserOdometry src/laserOdometry.cpp)
ament_target_dependencies(alaserOdometry
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  Eigen3
)
target_link_libraries(alaserOdometry
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ceres
  glog::glog
)

# 3. 激光建图
add_executable(alaserMapping src/laserMapping.cpp)
ament_target_dependencies(alaserMapping
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  Eigen3
)
target_link_libraries(alaserMapping
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ceres
  glog::glog
)


# ==============================================
# 安装目标
# ==============================================
install(TARGETS
  ascanRegistration
  alaserOdometry
  alaserMapping
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# 安装 launch 与 rviz 配置
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz_cfg DESTINATION share/${PROJECT_NAME})

# 导出包含目录
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
  rosbag2_cpp
  Eigen3
  OpenCV
  Ceres
  PCL
  glog
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()