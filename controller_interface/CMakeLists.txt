cmake_minimum_required(VERSION 3.8)
project(your_controller_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable)
endif()

find_package(PkgConfig REQUIRED)
# Add your dependencies here
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  generate_parameter_library
  parameter_traits
  backward_ros
  realtime_tools
  std_msgs
)


find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. Please install or update ros2_control.")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME} SHARED
  src/your_controller_name.cpp
  )

target_compile_features(your_controller_name PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(your_controller_name PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
generate_parameter_library(your_controller_name_parameters src/your_controller_name_parameters.yaml)
target_link_libraries(${PROJECT_NAME} PUBLIC rclcpp::rclcpp your_controller_name_parameters)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Advertize the controller
pluginlib_export_plugin_description_file(controller_interface your_controller_name_plugin.xml)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME} your_controller_name_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
