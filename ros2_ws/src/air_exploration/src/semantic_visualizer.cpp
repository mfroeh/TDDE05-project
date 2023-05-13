#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <chrono>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "air_interfaces/srv/get_entities.hpp"
#include "air_interfaces/msg/entity.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace rclcpp;

using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using air_interfaces::msg::Entity;
using air_interfaces::srv::GetEntities;

const char *SEMANTIC_VISUALIZER_NODE_NAME = "semantic_visualizer";
const char *SEMANTIC_VISUALIZER_PUBLISH_TOPIC = "/semantic_sensor_visualizer";

class SemanticVisualizer : public Node
{
public:
    SemanticVisualizer() : Node{SEMANTIC_VISUALIZER_NODE_NAME}
    {
        publisher = create_publisher<MarkerArray>(SEMANTIC_VISUALIZER_PUBLISH_TOPIC, 10);
        timer = create_wall_timer(500ms, std::bind(&SemanticVisualizer::query_and_visualize, this));
        // TODO: Create client to database proxy
        client = create_client<GetEntities>("get_entities");
    }

private:
    Publisher<MarkerArray>::SharedPtr publisher;
    TimerBase::SharedPtr timer;
    Client<GetEntities>::SharedPtr client;

    void visualize(std::vector<Entity> entities)
    {
        MarkerArray arr{};
        publisher->publish(arr);
    }

    void query_and_visualize()
    {
        RCLCPP_INFO(get_logger(), "Starting query");

        auto request{std::make_shared<GetEntities::Request>()};
        client->async_send_request(request, [this](Client<GetEntities>::SharedFuture future)
                                   {
            RCLCPP_INFO(get_logger(), "Got response");
            auto entities{future.get()->entities};
            visualize(entities); });
    };
};

int main(int argc, char *argv[])
{
    init(argc, argv);
    spin(std::make_shared<SemanticVisualizer>());
    shutdown();
}
