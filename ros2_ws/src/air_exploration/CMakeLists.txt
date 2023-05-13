cmake_minimum_required(VERSION 3.11)
project(air_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.10.4
)
FetchContent_MakeAvailable(nlohmann_json)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(air_interfaces REQUIRED) # GetPosition
find_package(nav2_msgs REQUIRED) # Odometry
find_package(nav_msgs REQUIRED) # Slam toolbox
find_package(geometry_msgs REQUIRED) # Point

# Transformation
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find_package(irobot_create_msgs REQUIRED) # Drive to position

find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(visualization_msgs REQUIRED) # Visualization of frontiers
find_package(std_msgs REQUIRED) # Colors

find_package(TstML REQUIRED) # TST executors
find_package(std_srvs REQUIRED) # TST executors
find_package(irobot_create_msgs REQUIRED) # Undock executor

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

# tst_executor
add_executable(tst_executor src/tst_executor src/explore_executor src/frontier src/undock_executor) # Add new executable
target_include_directories(tst_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tst_executor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  tst_executor
  "rclcpp"
  "nav_msgs"
  "nav2_msgs"
  "rclcpp_action"
  "air_interfaces"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "std_msgs"
  "TstML"
  "std_srvs"
  "irobot_create_msgs"
)
target_link_libraries(tst_executor nlohmann_json TstMLExecutor)

# semantic visualizer
add_executable(semantic_visualizer src/semantic_visualizer)
target_include_directories(semantic_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(semantic_visualizer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  semantic_visualizer
  "rclcpp"
  "air_interfaces"
  "std_msgs"
  "visualization_msgs"
)


install(TARGETS position_service tst_executor semantic_visualizer
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
