cmake_minimum_required(VERSION 3.8)
project(air_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(air_interfaces REQUIRED) # GetPosition
find_package(nav2_msgs REQUIRED) # Odometry
find_package(nav_msgs REQUIRED) # Slam toolbox
find_package(geometry_msgs REQUIRED) # Point

find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find_package(irobot_create_msgs REQUIRED) # Drive to position

# add include dirs
include_directories(include)

# position_service
add_executable(position_service src/position_service.cpp)
target_include_directories(position_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(position_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  position_service
  "rclcpp"
  "air_interfaces"
  "nav2_msgs"
  "geometry_msgs"
)

# exploration_server
add_executable(exploration_server src/exploration_server src/frontier src/driver) # Add new executable
target_include_directories(exploration_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(exploration_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  exploration_server
  "rclcpp"
  "nav_msgs"
  "nav2_msgs"
  "rclcpp_action"
  "air_interfaces"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  # "irobot_create_msgs"
)

install(TARGETS position_service exploration_server
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

ament_package()
