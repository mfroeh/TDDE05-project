cmake_minimum_required(VERSION 3.8)
project(air_navigation_layers)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(air_lab_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)


set(dep_pkgs
    rclcpp
    builtin_interfaces
    nav2_costmap_2d
    pluginlib
    air_lab_interfaces
    tf2
    tf2_ros
    tf2_kdl
    geometry_msgs
    angles
    )




# === Build ===

add_library(${lib_name} SHARED
src/human_layer.cpp
#src/proxemic_layer.cpp
#src/passing_layer.cpp
)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
DESTINATION lib)


add_executable(human_layer src/human_layer.cpp)
target_include_directories(human_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(human_layer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  human_layer
  "pluginlib"
)

install(TARGETS human_layer
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# === Ament work ===

# pluginlib_export_plugin_description_file() installs human_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
