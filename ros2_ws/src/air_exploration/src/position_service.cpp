#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "air_interfaces/srv/get_position.hpp"

using namespace rclcpp;
using namespace std::placeholders;

using geometry_msgs::msg::Point;
using nav_msgs::msg::Odometry;
using ServiceT = air_interfaces::srv::GetPosition;

char const *POSITION_SERVICE_NODE = "position_service";
char const *POSITION_SERVICE_TOPIC = "get_position";

class PositionService : public Node
{
public:
  PositionService() : Node{POSITION_SERVICE_NODE}
  {
    odom_subscription = create_subscription<Odometry>("odom", 10, std::bind(&PositionService::handle_odom_msg, this, _1));
    service = create_service<ServiceT>(POSITION_SERVICE_TOPIC, std::bind(&PositionService::serve, this, _1, _2));
  }

private:
  Point last_pos;

  Subscription<Odometry>::SharedPtr odom_subscription;
  Service<ServiceT>::SharedPtr service;

  void handle_odom_msg(const Odometry::SharedPtr msg)
  {
    last_pos = msg->pose.pose.position;
    RCLCPP_INFO_STREAM(get_logger(), "Received position x: " << last_pos.x << " y: " << last_pos.y << " z: " << last_pos.z);
  }

  void serve(const ServiceT::Request::SharedPtr, ServiceT::Response::SharedPtr response)
  {
    RCLCPP_INFO(get_logger(), "Received get_position request");
    response->point = last_pos;
    RCLCPP_INFO(get_logger(), "Sent last_pos");
  }
};

int main(int argc, char **argv)
{
  init(argc, argv);
  spin(std::make_shared<PositionService>());
  shutdown();
}
