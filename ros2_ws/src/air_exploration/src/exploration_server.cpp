#include <thread>
#include <chrono>
#include <iostream>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros2_kdb_msgs/srv/query_database.hpp>
#include <air_simple_sim_msgs/msg/semantic_observation.hpp>

#include "air_interfaces/action/explore.hpp"
#include "air_interfaces/srv/get_position.hpp"
#include "exploration_constants.hpp"
#include "frontier.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace rclcpp;
using namespace rclcpp_action;
using namespace std::placeholders;

using ActionT = air_interfaces::action::Explore;
using GoalHandleT = ServerGoalHandle<ActionT>;
using QueryServiceT = ros2_kdb_msgs::srv::QueryDatabase;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;

// TODO: Replace with wilson
using nav2_msgs::action::NavigateToPose;
using nav_msgs::msg::OccupancyGrid;

using air_simple_sim_msgs::msg::SemanticObservation;

class ExplorationActionServer : public Node
{
public:
    using Self = ExplorationActionServer;

    ExplorationActionServer() : Node{EXPLORATION_SERVER_NODE}
    {
        server = create_server<ActionT>(
            this,
            EXPLORATION_ACTION_TOPIC,
            std::bind(&Self::handle_goal, this, _1, _2),
            std::bind(&Self::handle_cancel, this, _1),
            std::bind(&Self::handle_accepted, this, _1));

        // The current map occupancy grid
        map_sub = create_subscription<OccupancyGrid>("map", 10, std::bind(&Self::handle_map, this, _1));
        odom_sub = create_subscription<Odometry>("odom", 10, std::bind(&Self::handle_odom, this, _1));

        // TODO: Replace with wilson
        navigate_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Transforming
        tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Database and semantic observation
        query_client = create_client<QueryServiceT>(QUERY_SERVICE_TOPIC);
        semantic_sub = create_subscription<SemanticObservation>("semantic_observation", 10, std::bind(&Self::handle_observation, this, _1));

        // Make sure all services are available
        assert(query_client->wait_for_service(std::chrono::seconds(1)));
        // assert(navigate_client->wait_for_service(std::chrono::seconds(1)));
        // Point p;
        // exists_in_database("", "", &p);
    }

private:
    Server<ActionT>::SharedPtr server{};

    Subscription<OccupancyGrid>::SharedPtr map_sub{};
    Subscription<Odometry>::SharedPtr odom_sub{};
    OccupancyGrid::SharedPtr map{};
    Point pos{};

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client{};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer{};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{};

    rclcpp::Client<QueryServiceT>::SharedPtr query_client{};
    Subscription<SemanticObservation>::SharedPtr semantic_sub{};

    /// @brief Handles incoming goal requests
    /// @param uuid The unique identifier of the goal
    /// @param goal The goal request
    /// @return Whether the goal should be accepted and executed
    GoalResponse handle_goal(const GoalUUID &uuid, std::shared_ptr<const ActionT::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received goal request for %s (%s)", goal->name.c_str(), goal->kind.c_str());
        return GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /// @brief Cancels the goal
    /// @param goal_handle The goal handle to cancel
    /// @return Whether the goal was successfully cancelled
    CancelResponse handle_cancel(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        // TODO: Maybe allow cancel
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return CancelResponse::REJECT;
    }

    /// @brief Begins working on newly created goal
    /// @param goal_handle The goal handle to work on
    void handle_accepted(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        check_exists_database(
            goal_handle->get_goal()->kind,
            goal_handle->get_goal()->name,
            [&](bool exists, Point p)
            {
                if (exists)
                {
                    auto result{std::make_shared<ActionT::Result>()};
                    result->success = true;
                    result->position = p;
                    goal_handle->succeed(result);
                }
                else
                {
                    // TODO: Should keep track of threads and cancel goal if thread is still running
                    std::thread{std::bind(&Self::explore, this, _1), goal_handle}.detach();
                }
            });
    }

    /// @brief Updates the occupation grid map
    /// @param msg The occupancy grid message
    void handle_map(const OccupancyGrid::SharedPtr msg)
    {
        map = msg;
        RCLCPP_INFO(get_logger(), "Updated map");
    }

    /// @brief Updates the current position
    /// @param msg The odometry message
    void handle_odom(const Odometry::SharedPtr msg)
    {
        pos = msg->pose.pose.position;
        RCLCPP_INFO_STREAM(get_logger(), "Received position x: " << pos.x << " y: " << pos.y << " z: " << pos.z);
    }

    /// @brief Keeps exploring frontiers until either target found or there are no more frontiers
    /// @param goal_handle The goal handle to work on
    void explore(const std::shared_ptr<GoalHandleT> goal_handle)
    {
        assert(map != nullptr);

        // TODO: Use a member variables which is set by the result callback (checks for person in database)
        bool found{};
        while (!found)
        {
            // TODO: Sort them
            std::vector<CellFrontier> frontiers{WFD(pos, Map{map})};
            CellFrontier cf{frontiers.front()};
            Frontier cur{std::move(cf), Map{map}};
            Point p{cur.centroid};

            // TODO: Go to the frontier using wilsons service
            NavigateToPose::Goal msg{};
            msg.pose.pose.position = p;
            msg.pose.header.frame_id = "map";

            rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options{};
            send_goal_options.goal_response_callback = std::bind(&Self::handle_drive_response, this, _1);
            send_goal_options.feedback_callback = std::bind(&Self::handle_drive_feedback, this, _1, _2);
            send_goal_options.result_callback = std::bind(&Self::handle_drive_result, this, _1);

            navigate_client->async_send_goal(msg, send_goal_options);
            RCLCPP_INFO(get_logger(), "Moving to frontier at x: %f, y: %f", p.x, p.y);
        }
    }

    void handle_drive_response(
        std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future)
    {
        auto todo = future.get();
        if (!todo)
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        else
            RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }

    void handle_drive_feedback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal,
                               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Remaining distance to goal: %f (%ds)", feedback->distance_remaining, feedback->estimated_time_remaining.sec);
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

    // TODO: This is copied from the lab
    // Maybe just keep track locally?
    void handle_observation(const SemanticObservation::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Got new observation!");
        auto request{std::make_shared<QueryServiceT::Request>()};
        request->graphname = GRAPHNAME;

        std::ostringstream os{};
        os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl
           << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>" << std::endl
           << "SELECT ?x ?y WHERE { <" << msg->uuid.c_str() << "> a <" << msg->klass.c_str() << "> ;" << std::endl
           << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;
        request->query = os.str();

        RCLCPP_INFO(get_logger(), "Query: %s", request->query.c_str());
        auto result = query_client->async_send_request(
            request,
            [this, msg](rclcpp::Client<QueryServiceT>::SharedFuture future)
            {
                RCLCPP_INFO(get_logger(), "%d\n",
                            future.get()->success);

                // QJsonDocument doc = QJsonDocument::fromJson(
                //     QByteArray::fromStdString(future.get()->result));

                // RCLCPP_INFO(get_logger(), "Json: %s", doc.toJson().toStdString().c_str());
                // if (doc.object()["results"].toObject()["bindings"].toArray().size() ==
                //     0)
                // {
                //     // TODO
                //     RCLCPP_INFO(get_logger(), "Doesn't exist!");
                // }
            });
    }

    /// @brief Checks if an object exists in the database
    /// @param kind The kind of the object
    /// @param name The name of the object
    /// @param callback The callback to call with the result
    template <typename CallbackT>
    void check_exists_database(std::string kind, std::string name, CallbackT callback)
    {
        RCLCPP_INFO(this->get_logger(), "Starting query\n");
        auto request{
            std::make_shared<QueryServiceT::Request>()};
        request->graphname = GRAPHNAME;

        std::ostringstream os{};
        os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl
           << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>" << std::endl
           << "SELECT ?obj_id ?class ?x ?y WHERE { ?obj_id a ?class ;" << std::endl
           << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;

        request->query = os.str();
        request->format = "json";

        query_client->async_send_request(
            request,
            [this, callback](
                rclcpp::Client<QueryServiceT>::SharedFuture future)
            {
                if (!future.get()->success)
                {
                    RCLCPP_ERROR(get_logger(), "Query was unsuccessful");
                    callback(false, Point{});
                    return;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "%s", future.get()->result.c_str());
                    // TODO: Set point
                    Point p{};
                    callback(true, p);
                }
            });
    }
};

int main(int argc, char const *argv[])
{
    init(argc, argv);
    spin(std::make_shared<ExplorationActionServer>());
    shutdown();
}