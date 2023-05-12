#include <memory>
#include <string>
#include <vector>

// kdb
#include "air_simple_sim_msgs/msg/semantic_observation.hpp"
#include "ros2_kdb_msgs/srv/insert_triples.hpp"
#include "ros2_kdb_msgs/srv/query_database.hpp"

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

using std::placeholders::_1;
using air_simple_sim_msgs::msg::SemanticObservation;
using QueryServiceT = ros2_kdb_msgs::srv::QueryDatabase;
using InsertServiceT = ros2_kdb_msgs::srv::InsertTriples;
using json = nlohmann::json;
using geometry_msgs::msg::PointStamped;

class DatabaseSubscriber : public rclcpp::Node
{
    public:
        DatabaseSubscriber() : Node("database_subscriber")
    {
        std::string topic {"/semantic_sensor"};
        subscription = this->create_subscription<SemanticObservation>(topic, 10, std::bind(&DatabaseSubscriber::semantic_callback, this, _1));
        query_client = this->create_client<QueryServiceT>("/kdb_server/sparql_query");
        insert_client = this->create_client<InsertServiceT>("/kdb_server/insert_triples");
        query_client->wait_for_service();
        insert_client->wait_for_service();
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    private:
        // Callback called with a semantic observation, checks if it's already present in DB and if not, adds it
        void semantic_callback(SemanticObservation const& msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received observation");

            std::string graph_name{"semanticobject"};

            insert_if_unique(graph_name, msg);
            queryAll(graph_name);
        }

        void insert(std::string const& graph_name, SemanticObservation const& msg) {

            PointStamped transformed_point{};
            try {
                transformed_point = tf_buffer->transform(msg.point, "map");
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                        msg.point.header.frame_id.c_str(), "map", ex.what());
                return;
            }

            std::ostringstream os{};
            os << "@prefix gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl;
            os << "@prefix properties: <http://www.ida.liu.se/~TDDE05/properties>"
                << std::endl;
            os << "<" << msg.uuid.c_str() << "> a <" << msg.klass.c_str() << ">;"
                << std::endl;
            os << "properties:location [ gis:x " << transformed_point.point.x << "; gis:y "
                << transformed_point.point.y << " ] ." << std::endl;

            RCLCPP_INFO(this->get_logger(), "Inserting - uuid: %s, klass: %s, x: %f, y: %f\n", msg.uuid.c_str(), msg.klass.c_str(), transformed_point.point.x, transformed_point.point.y);

            auto request{std::make_shared<InsertServiceT::Request>()};
            request->graphname = graph_name;
            request->format = "ttl";
            request->content = os.str();
            auto result = insert_client->async_send_request(
                    request, [this](rclcpp::Client<InsertServiceT>::SharedFuture future) {
                    RCLCPP_INFO(this->get_logger(), "Inserted: %s\n",
                            future.get()->success ? "Success" : "Failure");
                    RCLCPP_INFO(this->get_logger(), "Insert Future: %s\n",
                            future.get()->err_msgs.c_str());
                    });

        }

        void insert_if_unique(std::string const& graph_name, SemanticObservation const& msg) {

            auto request{std::make_shared<QueryServiceT::Request>()};
            request->graphname = graph_name;

            std::ostringstream os{};
            os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl
                << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
                << std::endl
                << "SELECT ?x ?y WHERE { <" << msg.uuid.c_str() << "> a <"
                << msg.klass.c_str() << "> ;" << std::endl;
            os << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;
            request->query = os.str();

            auto result = query_client->async_send_request(request, [this, graph_name, msg](rclcpp::Client<QueryServiceT>::SharedFuture future) {
                    if(future.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Database query successfull");

                    auto parsed_result{json::parse(future.get()->result)};
                    if(parsed_result["results"]["bindings"].size() == 0) {
                    RCLCPP_INFO(this->get_logger(), "Found new element! Inserting into database...");
                    insert(graph_name, msg);
                    }
                    }
                    });
        }


        void queryAll(std::string const& graph_name) {

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

            auto response = query_client->async_send_request(request, [this](rclcpp::Client<QueryServiceT>::SharedFuture future) {

                    RCLCPP_INFO(this->get_logger(), "Received result\n");
                    auto parsed_result{json::parse(future.get()->result)};

                    RCLCPP_INFO(this->get_logger(), "Object: %s\n", future.get()->result.c_str());

                    if (!future.get()->success) {
                    RCLCPP_ERROR(get_logger(),
                            "Query: /kdb_server/sparql_query wasn't successful!");
                    return;
                    }

                    std::vector<Object> ret{};
                    for(auto&& obj : parsed_result["results"]["bindings"]) {
                        Object temp{
                            obj["obj_id"]["value"].get<std::string>(),
                            obj["class"]["value"].get<std::string>(),
                            obj["x"]["value"].get<double>(),
                            obj["y"]["value"].get<double>()
                        };
                        ret.push_back(temp);
                    }

                    //TODO: do something with ret
                    });
        }


        rclcpp::Subscription<SemanticObservation>::SharedPtr subscription;
        rclcpp::Client<QueryServiceT>::SharedPtr query_client;
        rclcpp::Client<InsertServiceT>::SharedPtr insert_client;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        struct Object {
            std::string uuid;
            std::string klass;
            double x;
            double y;
        };


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DatabaseSubscriber>());
    rclcpp::shutdown();
    return 0;
}