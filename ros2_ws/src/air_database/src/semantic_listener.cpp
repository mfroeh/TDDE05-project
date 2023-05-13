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

using air_simple_sim_msgs::msg::SemanticObservation;
using std::placeholders::_1;
using QueryServiceT = ros2_kdb_msgs::srv::QueryDatabase;
using InsertServiceT = ros2_kdb_msgs::srv::InsertTriples;
using json = nlohmann::json;
using geometry_msgs::msg::PointStamped;

class SemanticListener : public rclcpp::Node
{
public:
    SemanticListener() : Node("semantic_listener")
    {
        std::string topic{"/semantic_sensor"};
        subscription = this->create_subscription<SemanticObservation>(topic, 10, std::bind(&SemanticListener::semantic_callback, this, _1));
        query_client = this->create_client<QueryServiceT>("/kdb_server/sparql_query");
        insert_client = this->create_client<InsertServiceT>("/kdb_server/insert_triples");
        query_client->wait_for_service();
        insert_client->wait_for_service();
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

private:
    // Callback called with a semantic observation, checks if it's already present in DB and if not, adds it
    void semantic_callback(SemanticObservation const &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received observation");

        std::string graph_name{"semanticobject"};

        insert_if_unique(graph_name, msg);
    }

    void insert(std::string const &graph_name, SemanticObservation const &msg)
    {

        PointStamped transformed_point{};
        try
        {
            transformed_point = tf_buffer->transform(msg.point, "map");
        }
        catch (const tf2::TransformException &ex)
        {
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
           << transformed_point.point.y << " ]" << std::endl;
        for(auto&& tag : msg.tags)
        {
            os << ";";
            os << "properties:tags <" + tag + ">" << std::endl;
        }
        os << "." << std::endl;

        RCLCPP_INFO(this->get_logger(), "Inserting - uuid: %s, klass: %s, x: %f, y: %f\n", msg.uuid.c_str(), msg.klass.c_str(), transformed_point.point.x, transformed_point.point.y);

        auto request{std::make_shared<InsertServiceT::Request>()};
        request->graphname = graph_name;
        request->format = "ttl";
        request->content = os.str();
        auto result = insert_client->async_send_request(
            request, [this](rclcpp::Client<InsertServiceT>::SharedFuture future)
            { RCLCPP_INFO(this->get_logger(), "Insertion result: %s\n",
                          future.get()->success ? "Success" : "Failure"); });
    }

    void insert_if_unique(std::string const &graph_name, SemanticObservation const &msg)
    {

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

        auto result = query_client->async_send_request(request, [this, graph_name, msg](rclcpp::Client<QueryServiceT>::SharedFuture future)
                                                       {
                    if(future.get()->success) {
                    auto parsed_result{json::parse(future.get()->result)};
                    if(parsed_result[0]["results"]["bindings"].size() == 0) {
                    insert(graph_name, msg);
                    }
                    } });
    }

    rclcpp::Subscription<SemanticObservation>::SharedPtr subscription;
    rclcpp::Client<QueryServiceT>::SharedPtr query_client;
    rclcpp::Client<InsertServiceT>::SharedPtr insert_client;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    struct Object
    {
        std::string uuid;
        std::string klass;
        double x;
        double y;
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SemanticListener>());
    rclcpp::shutdown();
    return 0;
}
