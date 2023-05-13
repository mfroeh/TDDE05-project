#include <string>
#include <vector>

// kdb
#include "ros2_kdb_msgs/srv/query_database.hpp"

// placeholders
#include <functional>

// json
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using QueryServiceT = ros2_kdb_msgs::srv::QueryDatabase;
using json = nlohmann::json;

class DatabaseProxy : public rclcpp::Node {
public:
  DatabaseProxy() : Node("database_proxy") {
    query_client =
        this->create_client<QueryServiceT>("/kdb_server/sparql_query");
    query_client->wait_for_service();
    std::string graph_name{"semanticobject"};
    queryAll(graph_name);
  }

private:
  struct Object {
    std::string uuid;
    std::string klass;
    double x;
    double y;
  };

  std::vector<Object> queryAll(std::string const& graph_name) {

    RCLCPP_INFO(this->get_logger(), "Starting query\n");

    auto request{std::make_shared<QueryServiceT::Request>()};
    request->graphname = graph_name;

    std::ostringstream os{};
    os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl;
    os << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
       << std::endl;
    os << "SELECT ?obj_id ?class ?x ?y WHERE { ?obj_id a ?class ;" << std::endl;
    os << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;

    request->query = os.str();
    request->format = "json";

    auto future = query_client->async_send_request(request);

    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    RCLCPP_INFO(this->get_logger(), "Received result");

    auto parsed_result{json::parse(future.get()->result)};

    RCLCPP_INFO(this->get_logger(), "Object: %s\n",
                future.get()->result.c_str());

    std::vector<Object> ret{};

    if (!future.get()->success) {
      RCLCPP_ERROR(get_logger(),
                   "Query: /kdb_server/sparql_query wasn't successful!");
      return ret;
    }

    auto bindings = parsed_result[0]["results"]["bindings"];
    for (auto&& obj : bindings) {
      Object temp{obj["obj_id"]["value"].get<std::string>(),
                  obj["class"]["value"].get<std::string>(),
                  stod(obj["x"]["value"].get<std::string>()),
                  stod(obj["y"]["value"].get<std::string>())};
      ret.push_back(temp);
    }
    return ret;
  }

  rclcpp::Client<QueryServiceT>::SharedPtr query_client;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<DatabaseProxy>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
