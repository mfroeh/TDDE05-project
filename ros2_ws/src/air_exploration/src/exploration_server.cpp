#include <thread>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "air_interfaces/action/explore.hpp"
#include "exploration_constants.hpp"
#include "map.hpp"

using namespace rclcpp;
using namespace rclcpp_action;
using namespace std::placeholders;

using ActionT = air_interfaces::action::Explore;
using GoalHandleT = ServerGoalHandle<ActionT>;

using nav_msgs::msg::OccupancyGrid;

const char *NODE_NAME = "exploration_server";
const char *ACTION_NAME = "exploration";

class ExplorationActionServer : public Node
{
public:
    using Self = ExplorationActionServer;

    ExplorationActionServer() : Node{EXPLORATION_SERVER_NODE}, map{0, 0}
    {
        server = create_server<ActionT>(
            this,
            EXPLORATION_ACTION_TOPIC,
            std::bind(&Self::handle_goal, this, _1, _2),
            std::bind(&Self::handle_cancel, this, _1),
            std::bind(&Self::handle_accepted, this, _1));

        map_sub = create_subscription<OccupancyGrid>("map", 10, std::bind(&Self::map_callback, this, _1));
    }

private:
    Server<ActionT>::SharedPtr server;
    Subscription<OccupancyGrid>::SharedPtr map_sub;
    Map map;

    /// @brief Handles incoming goal requests
    /// @param uuid
    /// @param goal
    /// @return
    GoalResponse handle_goal(const GoalUUID &uuid, std::shared_ptr<const ActionT::Goal> goal)
    {
        RCLCPP_DEBUG(get_logger(), "Received goal request");
        return GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /// @brief Cancels existing goals
    /// @param goal_handle
    /// @return
    CancelResponse handle_cancel(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        RCLCPP_DEBUG(get_logger(), "Received request to cancel goal");
        return CancelResponse::ACCEPT;
    }

    /// @brief Begins working on newly created goal
    /// @param goal_handle
    void handle_accepted(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        // TODO: Should keep track of threads and cancel goal if thread is still running
        std::thread{std::bind(&Self::begin_explore, this, _1), goal_handle}.detach();
    }

    void map_callback(const OccupancyGrid::SharedPtr msg)
    {
        map.update(msg);
        RCLCPP_INFO_STREAM(get_logger(), map);
    }

    // Go to the nearest frontier
    void begin_explore(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        for (;;)
        {
            std::cout << "Test" << std::endl;
            RCLCPP_DEBUG(get_logger(), "HI");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};

int main(int argc, char const *argv[])
{
    init(argc, argv);
    spin(std::make_shared<ExplorationActionServer>());
    shutdown();
}
