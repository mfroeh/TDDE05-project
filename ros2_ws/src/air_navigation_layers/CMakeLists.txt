cmake_minimum_required(VERSION 3.5)
project(air_navigation_layers)


set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(air_interfaces REQUIRED)
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
    air_interfaces
    tf2
    tf2_ros
    tf2_kdl
    geometry_msgs
    angles
    )

# === Build ===

add_library(${lib_name} SHARED
            src/human_layer.cpp
            src/proxemic_layer.cpp
            src/passing_layer.cpp
            )
include_directories(include)


# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)


# === Ament work ===

# pluginlib_export_plugin_description_file() installs human_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()