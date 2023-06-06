#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <QPoint>
#include <functional>
#include <future>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <memory>
#include <nav2_msgs/action/detail/navigate_to_pose__struct.hpp>
#include <nav2_msgs/msg/detail/speed_limit__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <fstream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "frontier.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>
#include <vector>

using nav_msgs::msg::OccupancyGrid;

class ExploreExecutorLab : public TstML::Executor::AbstractNodeExecutor
{
public:
    using Explore = nav2_msgs::action::NavigateToPose;
    using GoalHandleExplore = rclcpp_action::ClientGoalHandle<Explore>;

    ExploreExecutorLab(const TstML::TSTNode *_node,
                       TstML::Executor::AbstractExecutionContext *_context)
        : TstML::Executor::AbstractNodeExecutor(_node, _context)
    {
        static int counter{};
        m_node =
            rclcpp::Node::make_shared(("explore_node_lab") + std::to_string(++counter));
        m_executor.add_node(m_node);
        m_executor_thread = std::thread([this]()
                                        { m_executor.spin(); });
        m_client_ptr =
            rclcpp_action::create_client<Explore>(m_node, "navigate_to_pose");

        subscription = m_node->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&ExploreExecutorLab::odom_callback, this,
                      std::placeholders::_1));

        map_sub = m_node->create_subscription<OccupancyGrid>("map", 10, std::bind(&ExploreExecutorLab::handle_map, this, std::placeholders::_1));
    }

    ~ExploreExecutorLab()
    {
        m_executor.cancel();
        m_executor_thread.join();
    }

    void handle_map(OccupancyGrid::SharedPtr msg)
    {
        map = msg;
        RCLCPP_INFO(m_node->get_logger(), "Map: %p", map);
    }

    TstML::Executor::ExecutionStatus start() override
    {
        using namespace std::placeholders;
        if (!hasWaypoints)
            return TstML::Executor::ExecutionStatus::Started();

        auto goal_msg = Explore::Goal();

        if (map)
        {
            auto undiscovered{std::count(map->data.begin(), map->data.end(), -1)};
            unsigned long size{map->info.width * map->info.height};
            double rate{(size - undiscovered) / static_cast<double>(size)};
            RCLCPP_INFO(m_node->get_logger(), "Discovered %.2f%% (%lu/%lu) of map", rate * 100, size - undiscovered, size);

            bench << m_node->now().nanoseconds() << "," << waypoints.size() << "," << rate << std::endl;
        }
        else
        {
            bench << m_node->now().nanoseconds() << "," << waypoints.size() << ","
                  << "0.0" << std::endl;
        }

        bench.flush();

        auto front{waypoints.front()};
        waypoints.pop();
        goal_msg.pose.pose.position = front;
        goal_msg.pose.header.frame_id = "map";

        auto send_goal_options = rclcpp_action::Client<Explore>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ExploreExecutorLab::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ExploreExecutorLab::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ExploreExecutorLab::result_callback, this, _1);
        m_client_ptr->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Send goal! ");
        return TstML::Executor::ExecutionStatus::Started();
    }

    void goal_response_callback(
        std::shared_future<GoalHandleExplore::SharedPtr> future)
    {
        m_goal_handle = future.get();
        if (!m_goal_handle)
        {
            executionFinished(TstML::Executor::ExecutionStatus::Aborted());
            RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(m_node->get_logger(),
                        "Goal accepted by server, waiting for result");
        }
    }

    void
    feedback_callback(GoalHandleExplore::SharedPtr,
                      const std::shared_ptr<const Explore::Feedback> feedback)
    {
        // RCLCPP_INFO(m_node->get_logger(), "%f", feedback->distance_remaining);
        auto current_position{feedback->current_pose.pose.position};
        // RCLCPP_INFO(this->get_logger(), "Position: %f,%f", lp.x, lp.y);
        int cur_time{feedback->navigation_time.sec};
        bool same_position = std::abs(current_position.x - last_pos_.x) < 0.05 &&
                             std::abs(current_position.y - last_pos_.y) < 0.05;
        if (same_position && cur_time - time_of_last_change > 4)
        {
            m_client_ptr->async_cancel_all_goals();
        }
        else if (!same_position)
        {
            last_pos_ = current_position;
            time_of_last_change = feedback->navigation_time.sec;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (hasWaypoints)
            return;
        RCLCPP_INFO(m_node->get_logger(), "Got first odom message\n");

        double radius{node()->getParameter(TstML::TSTNode::ParameterType::Specific, "radius").toDouble()};
        double a{node()->getParameter(TstML::TSTNode::ParameterType::Specific, "a").toDouble()};
        double b{node()->getParameter(TstML::TSTNode::ParameterType::Specific, "b").toDouble()};
        constexpr double pi = 3.14159265358979323846;
        double increment{pi / 4};

        double theta{};
        double cur_r{a + b * theta};
        geometry_msgs::msg::Point pos = msg->pose.pose.position;
        while (cur_r < radius)
        {
            theta += increment;
            cur_r = a + b * theta;

            geometry_msgs::msg::Point p{};
            p.x = pos.x + std::cos(theta) * cur_r;
            p.y = pos.y + std::sin(theta) * cur_r;
            waypoints.push(p);
        }

        bench = std::ofstream{"bench" + std::to_string(m_node->now().nanoseconds()) + ".csv"};
        bench << "T,Size,Discovered" << std::endl;

        hasWaypoints = true;
        start();
    }

    void result_callback(const GoalHandleExplore::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(m_node->get_logger(), "Goal was succeeded");
            if (waypoints.size() != 0)
                start();
            else
                executionFinished(TstML::Executor::ExecutionStatus::Finished());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(m_node->get_logger(), "Goal was aborted");
            executionFinished(TstML::Executor::ExecutionStatus::Aborted());
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(m_node->get_logger(), "Goal was canceled");
            if (waypoints.size() != 0)
                start();
            else
            {
                executionFinished(TstML::Executor::ExecutionStatus::Aborted());
                return;
            }
            break;
        default:
            RCLCPP_ERROR(m_node->get_logger(), "Unknown result code");
            executionFinished(TstML::Executor::ExecutionStatus::Aborted());
            return;
        }
    }

    TstML::Executor::ExecutionStatus pause() override
    {
        return TstML::Executor::ExecutionStatus::Running();
    }
    TstML::Executor::ExecutionStatus resume() override
    {
        return TstML::Executor::ExecutionStatus::Running();
    }
    TstML::Executor::ExecutionStatus stop() override
    {
        m_client_ptr->async_cancel_goal(m_goal_handle);
        return TstML::Executor::ExecutionStatus::Finished();
    }
    TstML::Executor::ExecutionStatus abort() override
    {
        m_client_ptr->async_cancel_goal(m_goal_handle);
        return TstML::Executor::ExecutionStatus::Aborted();
    }

private:
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::executors::MultiThreadedExecutor m_executor;
    std::thread m_executor_thread;
    rclcpp_action::Client<Explore>::SharedPtr m_client_ptr;
    GoalHandleExplore::SharedPtr m_goal_handle;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
    std::queue<geometry_msgs::msg::Point> waypoints;
    geometry_msgs::msg::Point last_pos_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map{};
    int time_of_last_change;
    bool hasWaypoints{};
    std::ofstream bench;
};
