#include <string>
#include <vector>

// kdb
#include "air_simple_sim_msgs/msg/semantic_observation.hpp"

// placeholders
#include <functional>

// json
#include <nlohmann/json.hpp>

// transformation
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "air_interfaces/srv/get_entities.hpp"
#include "air_interfaces/msg/entity.hpp"

using air_simple_sim_msgs::msg::SemanticObservation;
using std::placeholders::_1;
using std::placeholders::_2;
using json = nlohmann::json;
using geometry_msgs::msg::PointStamped;
using GetEntitiesT = air_interfaces::srv::GetEntities;
using Entity = air_interfaces::msg::Entity;

template< typename tPair >
struct second_t {
    typename tPair::second_type operator()( const tPair& p ) const { return p.second; }
};

template< typename tMap >
second_t< typename tMap::value_type > second( const tMap& ) { return second_t< typename tMap::value_type >(); }

class SemanticListener : public rclcpp::Node {
public:
  SemanticListener() : Node("semantic_listener") {
    std::string topic{"/semantic_sensor"};
    subscription = this->create_subscription<SemanticObservation>(
        topic, 10, std::bind(&SemanticListener::semantic_callback, this, _1));
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    service = this->create_service<GetEntitiesT>(
        "get_entities", std::bind(&SemanticListener::get_entities, this, _1, _2));
  }

private:
  // Callback called with a semantic observation, checks if it's already present
  // in DB and if not, adds it
  void semantic_callback(SemanticObservation const& msg) {
    if (msg.klass != "human" && msg.klass != "vendingmachine" &&
        msg.klass != "office") {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Received observation");
    PointStamped transformed_point{};
    try {
      transformed_point = tf_buffer->transform(msg.point, "map");
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  msg.point.header.frame_id.c_str(), "map", ex.what());
      return;
    }

    Entity entity{};
    entity.uuid = msg.uuid;
    entity.klass = msg.klass;
    entity.tags = msg.tags;
    entity.x = transformed_point.point.x;
    entity.y = transformed_point.point.y;

    map.insert_or_assign(entity.uuid, entity);
  }

  void get_entities(std::shared_ptr<GetEntitiesT::Request> const,
                    std::shared_ptr<GetEntitiesT::Response> response) {
    std::transform(map.begin(), map.end(),
                   std::back_inserter(response->entities), second(map));
  }

  rclcpp::Subscription<SemanticObservation>::SharedPtr subscription;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::Service<GetEntitiesT>::SharedPtr service;
  std::map<std::string, Entity> map;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticListener>());
  rclcpp::shutdown();
  return 0;
}
