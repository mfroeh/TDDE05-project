#include "explore_executor.hpp"

#include <string>

char const *EXPLORE_EXECUTOR_NODE_NAME = "explore_node";

char const *QUERY_SERVICE_TOPIC = "/kdb_server/sparql_query";
char const *SEMANTIC_SENSOR_TOPIC = "/semantic_sensor";
char const *FRONTIER_VISUALIZATION_TOPIC = "frontier_visualization";
char const *GRAPHNAME = "semanticobject";

using namespace rclcpp;
using namespace rclcpp_action;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

using nav2_msgs::action::NavigateToPose;
using ros2_kdb_msgs::srv::QueryDatabase;

using visualization_msgs::msg::MarkerArray;

using Self = ExploreExecutor;

static double euclidean(Point a, Point b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

ExploreExecutor::ExploreExecutor(TstML::TSTNode const *tst_node, TstML::Executor::AbstractExecutionContext *context) : TstML::Executor::AbstractNodeExecutor{tst_node, context}
{
    using namespace std::placeholders;

    static int counter{};

    node = Node::make_shared(EXPLORE_EXECUTOR_NODE_NAME + std::to_string(++counter));
    executor.add_node(node);
    executor_thread = std::thread([this]()
                                  { executor.spin(); });

    // Map and odometry subscription
    map_sub = node->create_subscription<OccupancyGrid>("map", 10, std::bind(&Self::handle_map, this, _1));
    odom_sub = node->create_subscription<Odometry>("odom", 10, std::bind(&Self::handle_odom, this, _1));

    // Navigation client
    navigate_client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

    // Transformations
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Query database
    query_client = node->create_client<QueryDatabase>(QUERY_SERVICE_TOPIC);

    // Visualization
    visualization_pub = node->create_publisher<MarkerArray>(FRONTIER_VISUALIZATION_TOPIC, 10);

    // Make sure all services are available
    // assert(query_client->wait_for_service(std::chrono::seconds(1)));
    // assert(navigate_client->wait_for_service(std::chrono::seconds(1)));
}

ExploreExecutor::~ExploreExecutor()
{
    executor.cancel();
    executor_thread.join();
}

TstML::Executor::ExecutionStatus ExploreExecutor::start()
{

    // TODO: Try to get person from database and act accordingly

    assert(map != nullptr);
    explore_frontier(Map{map});
    return TstML::Executor::ExecutionStatus::Started();
}

TstML::Executor::ExecutionStatus ExploreExecutor::pause()
{
    return TstML::Executor::ExecutionStatus::Running();
}

TstML::Executor::ExecutionStatus ExploreExecutor::resume()
{
    return TstML::Executor::ExecutionStatus::Running();
}

TstML::Executor::ExecutionStatus ExploreExecutor::stop()
{
    // TODO
    // navigate_client->async_cancel_goal(goal_handle);
    return TstML::Executor::ExecutionStatus::Finished();
}

TstML::Executor::ExecutionStatus ExploreExecutor::abort()
{
    // TODO
    // navigate_client->async_cancel_goal(goal_handle);
    return TstML::Executor::ExecutionStatus::Aborted();
}

void ExploreExecutor::explore_frontier(Map map)
{
    using namespace std::placeholders;

    // Read in the parameters of the node
    std::string kind{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "kind").toString().toStdString()};
    std::string name{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "name").toString().toStdString()};
    std::string policy{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "policy").toString().toStdString()};
    unsigned minsize{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "minsize").toUInt()};

    RCLCPP_INFO(node->get_logger(), "Launching WFD");
    auto frontiers{WFD(Map{map}, minsize)};
    RCLCPP_INFO_STREAM(node->get_logger(), "Found " << frontiers.size() << " frontiers!");

    if (frontiers.empty())
    {
        RCLCPP_INFO(node->get_logger(), "No frontiers left to explore, aborting...");
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    }

    // Ascending by distance to robot
    auto nearest{[this](Frontier const &a, Frontier const &b)
                 { return euclidean(a.centroid, pos) < euclidean(b.centroid, pos); }};

    // Descending by size
    auto biggest{[this](Frontier const &a, Frontier const &b)
                 { return a.size > b.size; }};

    if (policy == "nearest")
        std::sort(frontiers.begin(), frontiers.end(), nearest);
    else if (policy == "biggest")
        std::sort(frontiers.begin(), frontiers.end(), biggest);
    else
        std::sort(frontiers.begin(), frontiers.end(), nearest);

    Point p{frontiers[0].centroid};
    visualize_frontier(frontiers);

    // TODO: Go to the frontier using wilsons service
    NavigateToPose::Goal msg{};
    msg.pose.pose.position = p;
    msg.pose.header.frame_id = "map";

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options{};
    send_goal_options.goal_response_callback = std::bind(&Self::handle_drive_response, this, _1);
    send_goal_options.feedback_callback = std::bind(&Self::handle_drive_feedback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Self::handle_drive_result, this, _1);
    navigate_client->async_send_goal(msg, send_goal_options);
    RCLCPP_INFO(node->get_logger(), "Started moving to frontier at x: %f, y: %f", p.x, p.y);
}

void ExploreExecutor::handle_map(OccupancyGrid::SharedPtr const msg)
{
    map = msg;
    // RCLCPP_INFO(node->get_logger(), "Updated map");
}

void ExploreExecutor::handle_odom(Odometry::SharedPtr const msg)
{
    PointStamped in{};
    in.header.frame_id = msg->header.frame_id;
    in.header.stamp = msg->header.stamp;
    in.point = msg->pose.pose.position;

    PointStamped out{};
    if (try_transform_to(in, out, "map", false))
    {
        pos = out.point;
        // RCLCPP_INFO(node->get_logger(), "Updated position");
    }
}

void ExploreExecutor::visualize_frontier(std::vector<Frontier> frontiers)
{
    using std_msgs::msg::ColorRGBA;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    MarkerArray arr{};
    Marker marker{};
    marker.id = 1242;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = 0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;

    for (auto &f : frontiers)
    {
        Point p{f.centroid};
        p.x = p.x;
        p.y = p.y;
        marker.points.push_back(p);

        ColorRGBA color;
        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.5;
        marker.colors.push_back(color);
    }
    arr.markers.push_back(marker);

    visualization_pub->publish(arr);
}

void ExploreExecutor::handle_drive_response(
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future)
{
    if (!future.get())
    {
        RCLCPP_ERROR(node->get_logger(), "Navigation was rejected, aborting goal...");
        // abort(goal_handle);
    }
    else
        RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
}

void ExploreExecutor::handle_drive_feedback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
                                            const std::shared_ptr<const NavigateToPose::Feedback>)
{
    // RCLCPP_INFO(node->get_logger(), "Remaining distance to goal: %f (%ds)", feedback->distance_remaining, feedback->estimated_time_remaining.sec);
}

void ExploreExecutor::handle_drive_result(rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult const &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Drive: Goal was succeeded");
        explore_frontier(Map{map});
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Drive: Goal was aborted: %d", static_cast<int>(result.code));
        explore_frontier(Map{map});
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Drive: Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        return;
    }
}

bool ExploreExecutor::try_transform_to(PointStamped in, PointStamped &out, std::string target, bool log) const
{
    try
    {
        // in.header.stamp = now() - Duration::from_seconds(0.1);
        out = tf_buffer->transform(in, target);
        if (log)
            RCLCPP_INFO(node->get_logger(), "Transformed from %s (%f, %f) to %s (%f, %f)", in.header.frame_id.c_str(), in.point.x, in.point.y, target.c_str(), out.point.x, out.point.y);
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        if (log)
            RCLCPP_INFO(node->get_logger(), "Error transforming pose from %s to %s: %s", in.header.frame_id.c_str(), target.c_str(), ex.what());
        return false;
    }
}