#include "explore_executor.hpp"

#include <chrono>
#include <string>
#include <algorithm>
#include <future>

#include "air_interfaces/msg/entity.hpp"

using namespace rclcpp;
using namespace rclcpp_action;
using namespace std::chrono_literals;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

using air_interfaces::srv::GetEntities;
using nav2_msgs::action::NavigateToPose;

using visualization_msgs::msg::MarkerArray;

using Self = ExploreExecutor;

char const *EXPLORE_EXECUTOR_NODE_NAME = "explore_node";
char const *GET_ENTRIES_SERVICE_TOPIC = "get_entities";
char const *FRONTIER_VISUALIZATION_TOPIC = "frontier_visualization";

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
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    navigate_client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose", callback_group);

    // Transformations
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Database entries
    entities_client = node->create_client<GetEntities>(GET_ENTRIES_SERVICE_TOPIC);

    // Visualization
    visualization_pub = node->create_publisher<MarkerArray>(FRONTIER_VISUALIZATION_TOPIC, 10);

    // Make sure all services are available
    assert(entities_client->wait_for_service(2s));
    assert(navigate_client->wait_for_action_server(2s));
}

ExploreExecutor::~ExploreExecutor()
{
    executor.cancel();
    executor_thread.join();
}

TstML::Executor::ExecutionStatus ExploreExecutor::start()
{
    while (!map)
        RCLCPP_INFO(node->get_logger(), "Waiting for map...");

    if (goal_entity_found())
    {
        RCLCPP_INFO(node->get_logger(), "The guy was already found!");
        executionFinished(TstML::Executor::ExecutionStatus::Finished());
        return TstML::Executor::ExecutionStatus::Finished();
    }

    generate_frontiers(Map{map});
    drive_to_next_frontier();
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
    navigate_client->async_cancel_goal(goal_handle);
    return TstML::Executor::ExecutionStatus::Finished();
}

TstML::Executor::ExecutionStatus ExploreExecutor::abort()
{
    navigate_client->async_cancel_goal(goal_handle);
    return TstML::Executor::ExecutionStatus::Aborted();
}

void ExploreExecutor::generate_frontiers(Map map)
{
    // Read in the parameters of the node
    std::string kind{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "kind").toString().toStdString()};
    std::string name{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "name").toString().toStdString()};
    std::string policy{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "policy").toString().toStdString()};
    unsigned minsize{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "minsize").toUInt()};

    RCLCPP_INFO(node->get_logger(), "Generating new frontiers with parameters: kind=%s, name=%s, policy=%s, minsize=%u", kind.c_str(), name.c_str(), policy.c_str(), minsize);

    RCLCPP_INFO(node->get_logger(), "Launching WFD");
    auto new_frontiers{WFD(Map{map}, minsize)};
    RCLCPP_INFO_STREAM(node->get_logger(), "Found " << new_frontiers.size() << " frontiers!");

    // Ascending by distance to robot
    auto nearest{[this](Frontier const &a, Frontier const &b)
                 { return euclidean(a.centroid, pos.point) < euclidean(b.centroid, pos.point); }};

    // Descending by size
    auto biggest{[this](Frontier const &a, Frontier const &b)
                 { return a.size > b.size; }};

    if (policy == "nearest")
        std::sort(new_frontiers.begin(), new_frontiers.end(), nearest);
    else if (policy == "biggest")
        std::sort(new_frontiers.begin(), new_frontiers.end(), biggest);
    else
        std::sort(new_frontiers.begin(), new_frontiers.end(), nearest);

    frontiers = {new_frontiers.begin(), new_frontiers.end()};

    auto undiscovered{std::count(map.get_data().begin(), map.get_data().end(), -1)};
    double rate{(map.size - undiscovered) / static_cast<double>(map.size)};
    RCLCPP_INFO(node->get_logger(), "Discovered %.2f%% (%lu/%lu) of map", rate * 100, map.size - undiscovered, map.size);
}

void ExploreExecutor::drive_to_next_frontier()
{
    using namespace std::placeholders;

    if (frontiers.empty())
    {
        RCLCPP_INFO(node->get_logger(), "No frontiers left to explore, aborting...");
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    }

    visualize_frontier(frontiers);

    Frontier f{frontiers.front()};
    frontiers.pop_front();
    Point p{f.centroid};

    current = std::make_unique<Frontier>(f);

    NavigateToPose::Goal msg{};
    msg.pose.pose.position = p;
    msg.pose.header.frame_id = "map";

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options{};
    send_goal_options.goal_response_callback = std::bind(&Self::handle_drive_response, this, _1);
    send_goal_options.feedback_callback = std::bind(&Self::handle_drive_feedback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Self::handle_drive_result, this, _1);
    navigate_client->async_send_goal(msg, send_goal_options);

    pos_snapshot = pos;
    timer = node->create_wall_timer(4s, std::bind(&Self::check_stuck, this));
    RCLCPP_INFO(node->get_logger(), "Started moving to frontier at x: %f, y: %f", p.x, p.y);
}

void ExploreExecutor::check_stuck()
{
    if (pos_snapshot.header.stamp != pos.header.stamp && euclidean(pos_snapshot.point, pos.point) < 0.025)
    {
        RCLCPP_INFO(node->get_logger(), "Drive: Robot is stuck, cancelling...");
        navigate_client->async_cancel_goal(goal_handle);
        timer->cancel();
    }
    else
    {
        pos_snapshot = pos;
    }
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
        pos = out;
        RCLCPP_INFO(node->get_logger(), "Updated position");
    }
}

void ExploreExecutor::visualize_frontier(std::deque<Frontier> frontiers)
{
    using std_msgs::msg::ColorRGBA;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    std::vector<Point> frontier_centroids{};
    std::transform(frontiers.begin(), frontiers.end(), std::back_inserter(frontier_centroids), [](Frontier const &f)
                   { return f.centroid; });

    Frontier all_points{std::accumulate(frontiers.begin(),
                                        frontiers.end(), Frontier{std::vector<unsigned>{}, Map{map}},
                                        [](Frontier &a, Frontier &b)
                                        {
                                            a.points.insert(a.points.end(), b.points.begin(), b.points.end());
                                            return a;
                                        })};

    // Create marker for frontier points
    Marker points_marker{};
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = node->now();
    points_marker.ns = "frontier_points";
    points_marker.id = 0;
    points_marker.type = Marker::SPHERE_LIST;
    points_marker.action = Marker::ADD;
    points_marker.scale.x = 0.2;
    points_marker.scale.y = 0.2;
    points_marker.scale.z = 0.2;
    points_marker.color.r = 1.0;
    points_marker.color.g = 0.65;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;
    points_marker.points = all_points.points;

    // Create marker for frontier centroids
    Marker centroids_marker{};
    centroids_marker.header.frame_id = "map";
    centroids_marker.header.stamp = node->now();
    centroids_marker.ns = "frontier_centroids";
    centroids_marker.id = 1;
    centroids_marker.type = Marker::CUBE_LIST;
    centroids_marker.action = Marker::ADD;
    centroids_marker.scale.x = 0.2;
    centroids_marker.scale.y = 0.2;
    centroids_marker.scale.z = 0.2;
    centroids_marker.color.r = 1.0;
    centroids_marker.color.g = 1.0;
    centroids_marker.color.b = 0.0;
    centroids_marker.color.a = 1.0;
    centroids_marker.points = frontier_centroids;

    // Create MarkerArray message and add markers
    MarkerArray marker_array{};
    marker_array.markers.push_back(centroids_marker);
    marker_array.markers.push_back(points_marker);
    visualization_pub->publish(marker_array);
}

void ExploreExecutor::handle_drive_response(
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future)
{
    goal_handle = future.get();
    if (!goal_handle)
    {
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        RCLCPP_ERROR(node->get_logger(), "Drive: Goal was rejected, aborting goal...");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Drive: Goal accepted by server, waiting for result");
    }
}

void ExploreExecutor::handle_drive_feedback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
                                            const std::shared_ptr<const NavigateToPose::Feedback>)
{
    // TODO: Implement timeout which cancels goal if havn't moved
    // RCLCPP_INFO(node->get_logger(), "Remaining distance to goal: %f (%ds)", feedback->distance_remaining, feedback->estimated_time_remaining.sec);
}

void ExploreExecutor::handle_drive_result(rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult const &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Drive: Goal was succeeded");
        if (goal_entity_found())
        {
            RCLCPP_INFO(node->get_logger(), "Drive: We found the guy. Finishing...");
            executionFinished(TstML::Executor::ExecutionStatus::Finished());
            return;
        }

        generate_frontiers(Map{map});
        drive_to_next_frontier();
        return;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Drive: Goal was aborted: %d", static_cast<int>(result.code));
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    case rclcpp_action::ResultCode::CANCELED:
    case rclcpp_action::ResultCode::UNKNOWN:
        RCLCPP_ERROR(node->get_logger(), "Drive: Goal was canceled");
        if (goal_entity_found())
        {
            RCLCPP_INFO(node->get_logger(), "Drive: We found the guy. Finishing...");
            executionFinished(TstML::Executor::ExecutionStatus::Finished());
            return;
        }

        if (euclidean(pos.point, current->centroid) < 0.25)
        {
            RCLCPP_INFO(node->get_logger(), "Drive: Goal was canceled, but we are close enough to the goal. Creating new frontiers...");
            generate_frontiers(Map{map});
        }

        drive_to_next_frontier();
        return;
    default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    }
}

bool ExploreExecutor::goal_entity_found()
{
    using air_interfaces::msg::Entity;

// #define CONTAINER
#ifdef CONTAINER
    return false;
#endif

    std::string kind{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "kind").toString().toStdString()};
    std::string name{TstML::Executor::AbstractNodeExecutor::node()->getParameter(TstML::TSTNode::ParameterType::Specific, "name").toString().toStdString()};

    auto future{entities_client->async_send_request(std::make_shared<GetEntities::Request>())};
    while (future.wait_for(1s) != std::future_status::ready)
    {
    }

    auto entries{future.get()->entities};
    return std::any_of(entries.begin(), entries.end(), [kind, name](Entity const &e)
                       { return e.klass == kind && e.tag == name; });
}

bool ExploreExecutor::try_transform_to(PointStamped in, PointStamped &out, std::string target, bool log) const
{
    try
    {
        in.header.stamp = node->now() - Duration::from_seconds(0.1);
        out = tf_buffer->transform(in, target);
        if (log)
            RCLCPP_INFO(node->get_logger(), "Transformed from %s (%f, %f) to %s (%f, %f)", in.header.frame_id.c_str(), in.point.x, in.point.y, target.c_str(), out.point.x, out.point.y);
        return true;
    }
    catch (tf2::TransformException const &ex)
    {
        if (log)
            RCLCPP_INFO(node->get_logger(), "Error transforming pose from %s to %s: %s", in.header.frame_id.c_str(), target.c_str(), ex.what());
        return false;
    }
}