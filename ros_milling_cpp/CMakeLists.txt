cmake_minimum_required(VERSION 3.8)
project(ros_milling_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(ros_milling_cpp src/ros_milling_cpp.cpp)
target_include_directories(ros_milling_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros_milling_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ros_milling_cpp
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(ros_milling_cpp_full src/ros_milling_cpp_full.cpp)
target_include_directories(ros_milling_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros_milling_cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ros_milling_cpp_full
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS ros_milling_cpp
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ros_milling_cpp_full
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY src/nc_program
  DESTINATION share/${PROJECT_NAME}/src/nc_program)
 
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
