cmake_minimum_required(VERSION 3.5)
project(extractor)

# find the package we want to use 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cae_microphone_array REQUIRED)

# include the directories
include_directories(include)

# add the executable files
add_executable(extractor src/extractor.cpp)

# link the libraries to the files
ament_target_dependencies(extractor rclcpp std_msgs sensor_msgs cae_microphone_array)

# 安装目标
install(TARGETS
  extractor
  DESTINATION lib/${PROJECT_NAME}
)

# 让 package 能够通过 ament_cmake 使用
ament_package()
