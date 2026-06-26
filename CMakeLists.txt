cmake_minimum_required(VERSION 3.8)
project(openarm_cartesian_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartesian_control_msgs REQUIRED)
find_package(openarm_hardware REQUIRED)
find_package(urdf REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(openarm_cartesian_control SHARED
  src/openarm_cartesian_controller.cpp
  src/cartesian_impedance.cpp
)

target_include_directories(openarm_cartesian_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(openarm_cartesian_control
  rclcpp
  rclcpp_action
  controller_interface
  hardware_interface
  pluginlib
  geometry_msgs
  cartesian_control_msgs
  openarm_hardware
  urdf
  eigen3_cmake_module
  Eigen3
  tf2_ros
  tf2_eigen
  realtime_tools
)

target_link_libraries(openarm_cartesian_control
  pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(controller_interface openarm_cartesian_control_plugins.xml)

install(TARGETS openarm_cartesian_control
  EXPORT export_openarm_cartesian_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
