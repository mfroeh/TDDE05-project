cmake_minimum_required(VERSION 3.8)
project(air_database)

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
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_kdb_msgs REQUIRED) # Database
find_package(air_simple_sim_msgs REQUIRED) # Semantic observation
find_package(air_interfaces REQUIRED)

add_executable(semantic_listener src/semantic_listener.cpp)
target_include_directories(semantic_listener PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(semantic_listener PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
    semantic_listener
    "rclcpp"
    "air_simple_sim_msgs"
    "tf2_ros"
    "tf2"
    "tf2_geometry_msgs"
    "geometry_msgs"
    "ros2_kdb_msgs"
    )
target_link_libraries(semantic_listener nlohmann_json)

add_executable(database_proxy src/database_proxy.cpp)
target_include_directories(database_proxy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(database_proxy PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
    database_proxy
    "rclcpp"
    "ros2_kdb_msgs"
    "air_interfaces"
    )
target_link_libraries(database_proxy nlohmann_json)

install(TARGETS semantic_listener database_proxy
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
