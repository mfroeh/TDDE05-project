#pragma once
#include <thread>
#include <memory>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/Executor/ExecutionStatus.h>
#include <TstML/TSTNode.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "air_interfaces/srv/get_entities.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "frontier.hpp"

class ExploreExecutor : public TstML::Executor::AbstractNodeExecutor
{
public:
    ExploreExecutor(TstML::TSTNode const *tst_node, TstML::Executor::AbstractExecutionContext *context);
    ~ExploreExecutor();

    TstML::Executor::ExecutionStatus start() override;
    TstML::Executor::ExecutionStatus pause() override;
    TstML::Executor::ExecutionStatus resume() override;
    TstML::Executor::ExecutionStatus stop() override;
    TstML::Executor::ExecutionStatus abort() override;

private:
    std::deque<Frontier> frontiers{};
    std::unique_ptr<Frontier> current{};

    std::shared_ptr<rclcpp::Node> node{};
    rclcpp::executors::MultiThreadedExecutor executor{};
    std::thread executor_thread{};

    rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub{};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub{};
    nav_msgs::msg::OccupancyGrid::SharedPtr map{};
    geometry_msgs::msg::PointStamped pos{};

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_client{};
    rclcpp::CallbackGroup::SharedPtr callback_group{};
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle{};
    rclcpp::TimerBase::SharedPtr timer{};
    geometry_msgs::msg::PointStamped pos_snapshot{};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer{};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{};

    rclcpp::Client<air_interfaces::srv::GetEntities>::SharedPtr entities_client{};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub{};

private:
    void generate_frontiers(Map map);

    void drive_to_next_frontier();

    void check_stuck();

    /// @brief Updates the current position
    /// @param msg The odometry message
    void handle_odom(nav_msgs::msg::Odometry::SharedPtr const msg);

    /// @brief Updates the occupation grid map
    /// @param msg The occupancy grid message
    void handle_map(nav_msgs::msg::OccupancyGrid::SharedPtr const msg);

    /// @brief Visualizes the points of all the frontiers and their centroids
    /// @param frontiers
    void visualize_frontier(std::deque<Frontier> frontiers);

    void handle_drive_response(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future);

    void handle_drive_feedback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> const feedback);

    void handle_drive_result(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult const &result);

    bool goal_entity_found();

    /// @brief Tries to transform a point to a target frame
    /// @param in The point to transform
    /// @param out The transformed point
    /// @param target The target frame
    /// @return True if the transformation was successful, false otherwise
    bool try_transform_to(geometry_msgs::msg::PointStamped in, geometry_msgs::msg::PointStamped &out, std::string target, bool log) const;
};