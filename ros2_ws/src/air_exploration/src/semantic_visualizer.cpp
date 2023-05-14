#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <chrono>

#include <geometry_msgs/msg/point.hpp>
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
        client = create_client<GetEntities>("get_entities");
    }

private:
    Publisher<MarkerArray>::SharedPtr publisher;
    TimerBase::SharedPtr timer;
    Client<GetEntities>::SharedPtr client;

    void visualize(std::vector<Entity> entities)
    {
        using geometry_msgs::msg::Point;

        std::vector<Point> humans{}, vendingmachines{}, offices{};
        for (auto &e : entities)
        {
            Point p;
            p.x = e.x;
            p.y = e.y;
            if (e.klass == "human")
                humans.push_back(p);
            else if (e.klass == "vendingmachine")
                vendingmachines.push_back(p);
            else if (e.klass == "office")
                offices.push_back(p);
        }

        Marker humans_marker{};
        humans_marker.header.frame_id = "map";
        humans_marker.header.stamp = now();
        humans_marker.ns = "humans";
        humans_marker.id = 0;
        humans_marker.type = Marker::CUBE_LIST;
        humans_marker.action = Marker::ADD;
        humans_marker.scale.x = 0.2;
        humans_marker.scale.y = 0.2;
        humans_marker.scale.z = 0.2;
        humans_marker.color.r = 1.0;
        humans_marker.color.g = 1.0;
        humans_marker.color.b = 1.0;
        humans_marker.color.a = 1.0;
        humans_marker.points = humans;

        Marker vending_marker{};
        vending_marker.header.frame_id = "map";
        vending_marker.header.stamp = now();
        vending_marker.ns = "vendingmachines";
        vending_marker.id = 1;
        vending_marker.type = Marker::CUBE_LIST;
        vending_marker.action = Marker::ADD;
        vending_marker.scale.x = 0.2;
        vending_marker.scale.y = 0.2;
        vending_marker.scale.z = 0.2;
        vending_marker.color.r = 1.0;
        vending_marker.color.g = 0.0;
        vending_marker.color.b = 1.0;
        vending_marker.color.a = 1.0;
        vending_marker.points = vendingmachines;

        Marker offices_marker{};
        offices_marker.header.frame_id = "map";
        offices_marker.header.stamp = now();
        offices_marker.ns = "offices";
        offices_marker.id = 1;
        offices_marker.type = Marker::CUBE_LIST;
        offices_marker.action = Marker::ADD;
        offices_marker.scale.x = 0.2;
        offices_marker.scale.y = 0.2;
        offices_marker.scale.z = 0.2;
        offices_marker.color.r = 1.0;
        offices_marker.color.g = 0.0;
        offices_marker.color.b = 1.0;
        offices_marker.color.a = 1.0;
        offices_marker.points = offices;

        // Create MarkerArray message and add markers
        MarkerArray marker_array{};
        marker_array.markers.push_back(vending_marker);
        marker_array.markers.push_back(humans_marker);
        marker_array.markers.push_back(offices_marker);
        publisher->publish(marker_array);
    }

    void query_and_visualize()
    {
        RCLCPP_INFO(get_logger(), "Starting query");

        auto request{std::make_shared<GetEntities::Request>()};
        client->async_send_request(request, [this](Client<GetEntities>::SharedFuture future)
                                   {
            RCLCPP_INFO(get_logger(), "Got response");
            auto entities{future.get()->entities};
            RCLCPP_INFO(this->get_logger(), "Received %d entities", entities.size());
            visualize(entities); });
    };
};

int main(int argc, char *argv[])
{
    init(argc, argv);
    spin(std::make_shared<SemanticVisualizer>());
    shutdown();
}
