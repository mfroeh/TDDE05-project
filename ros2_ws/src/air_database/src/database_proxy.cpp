#include <future>
#include <string>
#include <vector>

// kdb
#include "ros2_kdb_msgs/srv/query_database.hpp"

// placeholders
#include <functional>

// json
#include <nlohmann/json.hpp>

#include "air_interfaces/msg/entity.hpp"
#include "air_interfaces/srv/get_entities.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using QueryServiceT = ros2_kdb_msgs::srv::QueryDatabase;
using json = nlohmann::json;
using Entity = air_interfaces::msg::Entity;
using GetEntityT = air_interfaces::srv::GetEntities;

class DatabaseProxy : public rclcpp::Node {
public:
  DatabaseProxy() : Node("database_proxy") {
    callback_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    query_client = this->create_client<QueryServiceT>(
        "/kdb_server/sparql_query", rmw_qos_profile_services_default,
        callback_group);
    query_client->wait_for_service();
    service = this->create_service<GetEntityT>(
        "get_entities", std::bind(&DatabaseProxy::get_entities, this, _1, _2));
  }

private:
  void get_entities(std::shared_ptr<GetEntityT::Request> const,
                    std::shared_ptr<GetEntityT::Response> response) {
    std::string graph_name{"semanticobject"};
    response->entities = query_all(graph_name);
  }

  std::vector<Entity> query_all(std::string const& graph_name) {

    RCLCPP_INFO(this->get_logger(), "Starting query\n");

    auto request{std::make_shared<QueryServiceT::Request>()};
    request->graphname = graph_name;

    std::ostringstream os{};
    os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>\n"
       << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>\n"
       << "SELECT ?obj_id ?class ?tags ?x ?y WHERE { ?obj_id a ?class ; "
       << "properties:location [ gis:x ?x; gis:y ?y ] ; properties:tags "
       << "?tags . }\n";

    request->query = os.str();
    request->format = "json";

    auto future = query_client->async_send_request(request);

    while (future.wait_for(1s) != std::future_status::ready)
      ;

    RCLCPP_INFO(this->get_logger(), "Received result");

    auto parsed_result{json::parse(future.get()->result)};

    std::vector<Entity> ret{};

    if (!future.get()->success) {
      RCLCPP_ERROR(get_logger(),
                   "Query: /kdb_server/sparql_query wasn't successful!");
      return ret;
    }

    auto bindings = parsed_result[0]["results"]["bindings"];
    for (auto&& obj : bindings) {
      Entity temp{};
      temp.uuid = obj["obj_id"]["value"].get<std::string>();
      temp.klass = obj["class"]["value"].get<std::string>();
      temp.tags = obj["tags"]["value"].get<std::vector<std::string>>();
      temp.x = stod(obj["x"]["value"].get<std::string>());
      temp.y = stod(obj["y"]["value"].get<std::string>());

      if (temp.klass == "human" || temp.klass == "vendingmachine" ||
          temp.klass == "office")
        ret.push_back(temp);
    }
    RCLCPP_INFO(this->get_logger(), "Sending %d objects",
                static_cast<int>(ret.size()));
    return ret;
  }

  rclcpp::Client<QueryServiceT>::SharedPtr query_client;
  rclcpp::Service<air_interfaces::srv::GetEntities>::SharedPtr service;
  rclcpp::CallbackGroup::SharedPtr callback_group;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<DatabaseProxy>();
  executor.add_node(node);
  executor.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
