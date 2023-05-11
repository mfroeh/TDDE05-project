#include "air_navigation_layers/human_layer.hpp"

#include <memory>
#include <functional>
#include <string>


#include "nav2_costmap_2d/costmap_layer.hpp"

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "air_interfaces/msg/people.hpp"
#include "air_interfaces/msg/person.hpp"

using air_interfaces::msg::Person;
using air_interfaces::msg::People;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;


namespace air_navigation_layers
{
    HumanLayer::HumanLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{}

void HumanLayer::onInitialize()
{
    current_ = true;
    first_time_ = true;
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    people_sub_ = node->create_subscription<People>(
        "/people", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&HumanLayer::peopleCallback, this, std::placeholders::_1)
    );
}

    void HumanLayer::peopleCallback(const People::SharedPtr people)
{
    boost::recursive_mutex::scoped_lock lock(lock_);
    auto node = node_.lock();
    if (!node) 
    {
        throw std::runtime_error{"Failed to lock node"};
    }
   
        if (people)
    {
        people_list_ = *people;
        
        //RCLCPP_INFO(node->get_logger(),"Received %ld people \n",people_list_.people.size());
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Received null people message.");
    }
    
} 

void HumanLayer::updateBounds(
    double /* robot_x */, double /* robot_y */, double /* robot_yaw */, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y)
{
    boost::recursive_mutex::scoped_lock lock(lock_);
    std::string global_frame = layered_costmap_->getGlobalFrameID();
    transformed_people_.clear();
    for(auto const& person : people_list_.people){
        Person tpt;
        geometry_msgs::msg::PointStamped pt, opt;
        try
        {
        pt.point.x = person.position.x;
        pt.point.y = person.position.y;
        pt.point.z = person.position.z;
        pt.header.frame_id = people_list_.header.frame_id;
        pt.header.stamp = people_list_.header.stamp;
        //pt.header.stamp = rclcpp::Clock().now();
        tf_->transform(pt, opt, global_frame);
        tpt.position.x = opt.point.x;
        tpt.position.y = opt.point.y;
        tpt.position.z = opt.point.z;

        pt.point.x += person.velocity.x;
        pt.point.y += person.velocity.y;
        pt.point.z += person.velocity.z;
        tf_->transform(pt, opt, global_frame);

        tpt.velocity.x = opt.point.x - tpt.position.x;
        tpt.velocity.y = opt.point.y - tpt.position.y;
        tpt.velocity.z = opt.point.z - tpt.position.z;
        
        tpt.name = person.name;

        transformed_people_.push_back(tpt);
        }
        catch (tf2::LookupException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
        RCLCPP_ERROR(node->get_logger(),"No Transform available Error: %s\n", ex.what());
        continue;
        }
        catch (tf2::ConnectivityException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            RCLCPP_ERROR(node->get_logger(),"Connectivity Error: %s\n", ex.what());
            continue;
        }
        catch (tf2::ExtrapolationException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            RCLCPP_ERROR(node->get_logger(),"Extrapolation Error: %s\n", ex.what());
            continue;
        }
  }    
    

    //updateBoundsFromPeople(min_x, min_y, max_x, max_y);
  if (first_time_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  }
  else
  {
    double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}

void HumanLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) return;
    auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }

    double buffer_size = 0.2; // Buffer size in meters
    double resolution = master_grid.getResolution();

    int buffer_cells = static_cast<int>(buffer_size / resolution);

    for(auto const& guy : transformed_people_)
    {
        double lx{guy.position.x}, ly{guy.position.y};
        unsigned int mx, my;
        if (!master_grid.worldToMap(lx, ly, mx, my)) {
            //RCLCPP_WARN(node->get_logger(), "Person is outside the costmap bounds, skipping.");
            continue;
        }

        for (int dx = -buffer_cells; dx <= buffer_cells; ++dx) {
            for (int dy = -buffer_cells; dy <= buffer_cells; ++dy) {
                unsigned int cell_x = mx + dx;
                unsigned int cell_y = my + dy;

                // Check if the cell is within the update window
                if (cell_x >= min_i && cell_x < max_i && cell_y >= min_j && cell_y < max_j) 
                {
                    master_grid.setCost(cell_x, cell_y, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }

        unsigned char cost{master_grid.getCost(mx, my)};
        RCLCPP_INFO(node->get_logger(),"Person:  %s in x: %lf, y: %lf, cost %u \n",guy.name.c_str(),lx,ly,cost);
    }
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(air_navigation_layers::HumanLayer, nav2_costmap_2d::Layer)
