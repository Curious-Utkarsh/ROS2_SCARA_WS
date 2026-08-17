cmake_minimum_required(VERSION 3.8)
project(scara_firmware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# LibSerial ships a pkg-config file rather than a CMake config package.
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

# ros2_control loads hardware interfaces as plugins, so this must be a SHARED
# library rather than an executable.
add_library(scara_hardware_interface SHARED
  src/scara_hardware_interface.cpp
  src/conveyor_relay.cpp
)

target_include_directories(scara_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIRS}
)

ament_target_dependencies(scara_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

target_link_libraries(scara_hardware_interface ${SERIAL_LDFLAGS})

# Registers scara_hardware_interface.xml with pluginlib, so ros2_control can
# resolve <plugin>scara_firmware/ScaraHardwareInterface</plugin> from the URDF.
pluginlib_export_plugin_description_file(hardware_interface scara_hardware_interface.xml)

install(TARGETS scara_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# The Arduino sketches are not compiled here -- they are installed so they are
# easy to find with `ros2 pkg prefix scara_firmware`.
install(DIRECTORY firmware docs
  DESTINATION share/${PROJECT_NAME}
)

# Standalone tuning GUI. It talks to the Arduino directly and does not use ROS,
# but installing it here means `ros2 run scara_firmware scara_gui.py` works.
install(PROGRAMS tools/scara_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(scara_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
