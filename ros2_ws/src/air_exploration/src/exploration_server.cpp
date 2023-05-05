#include <thread>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "air_interfaces/action/explore.hpp"
#include "exploration_constants.hpp"
#include "frontier.hpp"

using namespace rclcpp;
using namespace rclcpp_action;
using namespace std::placeholders;

using ActionT = air_interfaces::action::Explore;
using GoalHandleT = ServerGoalHandle<ActionT>;

using nav2_msgs::action::NavigateToPose;
using nav_msgs::msg::OccupancyGrid;

class ExplorationActionServer : public Node
{
public:
    using Self = ExplorationActionServer;

    ExplorationActionServer() : Node{EXPLORATION_SERVER_NODE}, algo{}
    {
        server = create_server<ActionT>(
            this,
            EXPLORATION_ACTION_TOPIC,
            std::bind(&Self::handle_goal, this, _1, _2),
            std::bind(&Self::handle_cancel, this, _1),
            std::bind(&Self::handle_accepted, this, _1));

        map_sub = create_subscription<OccupancyGrid>("map", 10, std::bind(&Self::map_callback, this, _1));

        driver = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    }

private:
    Server<ActionT>::SharedPtr server;
    Subscription<OccupancyGrid>::SharedPtr map_sub;
    rclcpp_action::Client<NavigateToPose>::SharedPtr driver;
    Algorithm algo;

    /// @brief Handles incoming goal requests
    /// @param uuid
    /// @param goal
    /// @return
    GoalResponse handle_goal(const GoalUUID &uuid, std::shared_ptr<const ActionT::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received goal request");
        return GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /// @brief Cancels existing goals
    /// @param goal_handle
    /// @return
    CancelResponse handle_cancel(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return CancelResponse::ACCEPT;
    }

    /// @brief Begins working on newly created goal
    /// @param goal_handle
    void handle_accepted(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        // TODO: Should keep track of threads and cancel goal if thread is still running
        std::thread{std::bind(&Self::explore, this, _1), goal_handle}.detach();
    }

    /// @brief Updates the occupation grid map
    /// @param msg
    void map_callback(const OccupancyGrid::SharedPtr msg)
    {
        algo.update_map(msg);
    }

    // Go to the nearest frontier
    void explore(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        if (algo.count() == 0)
        {
            RCLCPP_INFO(get_logger(), "No frontiers left to explore!\n");
            goal_handle->abort(std::make_shared<ActionT::Result>());
            return;
        }

        Frontier next{algo.pop()};
        Point p{next.center()};

        NavigateToPose::Goal msg{};
        msg.pose.pose.position = p;
        msg.pose.header.frame_id = "map";

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options{};
        send_goal_options.goal_response_callback = std::bind(&Self::handle_drive_response, this, _1);
        send_goal_options.feedback_callback = std::bind(&Self::handle_drive_feedback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&Self::handle_drive_result, this, _1);

        driver->async_send_goal(msg, send_goal_options);
        RCLCPP_INFO(get_logger(), "Moving to frontier at x: %f, y: %f", p.x, p.y);
    }

    void handle_drive_response(
        std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future)
    {
        auto todo = future.get();
        if (!todo)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void handle_drive_feedback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal,
                               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Remaining distance to goal: %f (%fs)", feedback->distance_remaining, feedback->estimated_time_remaining.sec);
    }

    void handle_drive_result(rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult const &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal was succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted: %d", (int)result.code);
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            return;
        }
    }
};

int main(int argc, char const *argv[])
{
    init(argc, argv);
    spin(std::make_shared<ExplorationActionServer>());
    shutdown();
}
