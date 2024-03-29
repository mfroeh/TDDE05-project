#include "air_navigation_layers/human_layer.hpp"

#include <memory>
#include <functional>
#include <algorithm>
#include <string>


#include "nav2_costmap_2d/costmap_layer.hpp"

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "air_interfaces/msg/people.hpp"
#include "air_interfaces/msg/person.hpp"

#include "nav_msgs/msg/path.hpp"

using air_interfaces::msg::Person;
using air_interfaces::msg::People;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Path;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

// Helper function to compute the Gaussian cost
double Gaussian_(double distance, double amplitude, double sigma) {
    return amplitude * std::exp(-(distance * distance) / (2 * sigma * sigma));
}





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
    first_time_costmap_=true;
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    people_sub_ = node->create_subscription<People>(
        "/people", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&HumanLayer::peopleCallback, this, std::placeholders::_1)
    );
    path_sub_ = node->create_subscription<Path>("/plan",rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&HumanLayer::planCallback, this, std::placeholders::_1)
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
        //people->header.stamp = rclcpp::Clock().now() - rclcpp::Duration::from_seconds(0.1);//make a later time
        people_list_ = *people;
        
        //RCLCPP_INFO(node->get_logger(),"[HUMAN_LAYER] Received %ld people \n",people_list_.people.size());
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Received null people message.");
    }
    
} 

    void HumanLayer::planCallback(const Path::SharedPtr path)
    {
        boost::recursive_mutex::scoped_lock lock(lock_);
    auto node = node_.lock();
    if (!node) 
    {
        throw std::runtime_error{"Failed to lock node"};
    }
    if(path)
    {
        path_list_=*path;
        RCLCPP_INFO(node->get_logger(),"[HUMAN_LAYER] Received %ld poses \n",path_list_.poses.size());
    }
    }

void HumanLayer::updateBounds(
    double robot_x, double robot_y, double /* robot_yaw */, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y)
{
    boost::recursive_mutex::scoped_lock lock(lock_);
    std::string global_frame = layered_costmap_->getGlobalFrameID();
    robot_x_=robot_x;
    robot_y_=robot_y;

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
        //pt.header.stamp = people_list_.header.stamp;
        pt.header.stamp = rclcpp::Clock().now();

        tf_->transform(pt, opt, global_frame);
        //opt=pt;

        tpt.position.x = opt.point.x;
        tpt.position.y = opt.point.y;
        tpt.position.z = opt.point.z;

        pt.point.x += person.velocity.x;
        pt.point.y += person.velocity.y;
        pt.point.z += person.velocity.z;

        tf_->transform(pt, opt, global_frame);
        //opt=pt;

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
        RCLCPP_ERROR(node->get_logger(),"[HUMAN_LAYER] No Transform available Error: %s\n", ex.what());
        continue;
        }
        catch (tf2::ConnectivityException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            RCLCPP_ERROR(node->get_logger(),"[HUMAN_LAYER] Connectivity Error: %s\n", ex.what());
            continue;
        }
        catch (tf2::ExtrapolationException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            RCLCPP_ERROR(node->get_logger(),"[HUMAN_LAYER] Extrapolation Error: %s\n", ex.what());
            continue;
        }
    }    
    
    future_path_.clear();
    for(auto const& pose : path_list_.poses)
    {
        Pose tft;
        geometry_msgs::msg::PointStamped ft, oft;
        try{
        ft.point.x = pose.pose.position.x;
        ft.point.y = pose.pose.position.y;
        ft.point.z = pose.pose.position.z;

        ft.header.frame_id = path_list_.header.frame_id;
        ft.header.stamp = rclcpp::Clock().now();

        tf_->transform(ft, oft, global_frame);

        tft.position.x = oft.point.x;
        tft.position.y = oft.point.y;
        tft.position.z = oft.point.z;

        future_path_.push_back(tft);
        }
        catch (tf2::LookupException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
        RCLCPP_ERROR(node->get_logger(),"[HUMAN_LAYER] No Transform available Error: %s\n", ex.what());
        continue;
        }
        catch (tf2::ConnectivityException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            RCLCPP_ERROR(node->get_logger(),"[HUMAN_LAYER] Connectivity Error: %s\n", ex.what());
            continue;
        }
        catch (tf2::ExtrapolationException& ex)
        {
            auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
            RCLCPP_ERROR(node->get_logger(),"[HUMAN_LAYER] Extrapolation Error: %s\n", ex.what());
            continue;
        }





    }

    auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
        RCLCPP_INFO(node->get_logger(),"[HUMAN_LAYER] We Know: %ld\n", transformed_people_.size());

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
/*     if (!enabled_)
    {
        auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }
        RCLCPP_WARN(node->get_logger(), "[HUMAN_LAYER] Not enabled, skipping.");
        return;
    } */

    if (first_time_costmap_)//save the map before implement human cost
  {
    init_map=master_grid;
    first_time_ = false;
  }
    else{//refresh the costmap into the init
        master_grid = init_map;
    }

    auto node = node_.lock();
            if (!node) {
                throw std::runtime_error{"Failed to lock node"};
            }

    unsigned int robot_mx, robot_my;
    master_grid.worldToMap(robot_x_, robot_y_, robot_mx, robot_my);

    double buffer_size = 5.0; // Buffer size in meters
    double inner_radius = 0.0; // Inner radius in meters
    double outer_radius = 1.5; // Outer radius in meters
    double resolution = master_grid.getResolution();

    int buffer_cells = static_cast<int>(buffer_size / resolution);
    //int outer_buffer_cells = static_cast<int>(outer_radius / resolution);

    double amplitude = nav2_costmap_2d::LETHAL_OBSTACLE-1; // Maximum cost value
    double sigma = 0.5; // Gaussian function spread

    for(auto const& guy : transformed_people_)
    {
        if(guy.velocity.x+guy.velocity.y+guy.velocity.z)//has velocity
        {
            outer_radius = 1.5;
        }else
        {
            outer_radius = 3.0;
        }
        int outer_buffer_cells = static_cast<int>(outer_radius / resolution);
        double lx{guy.position.x}, ly{guy.position.y};
        unsigned int person_mx, person_my;
        if (!master_grid.worldToMap(lx, ly, person_mx, person_my)) {  // GLOBAL_COSTMAP
            //RCLCPP_WARN(node->get_logger(), "Person is outside the costmap bounds, skipping.");
            continue;
        }
        bool has_obstacle = isBlocked(master_grid, person_mx, person_my, robot_mx, robot_my);
        if(!has_obstacle){
        for (int dx = -outer_buffer_cells; dx <= outer_buffer_cells; ++dx) {
            for (int dy = -outer_buffer_cells; dy <= outer_buffer_cells; ++dy) {
                unsigned int cell_x = person_mx + dx;
                unsigned int cell_y = person_my + dy;

                // Check if the cell is within the update window
                if (cell_x >= min_i && cell_x < max_i && cell_y >= min_j && cell_y < max_j) {
                    double distance = std::sqrt(dx * dx + dy * dy) * resolution;
                    
                    unsigned char cost;
                    if (distance <= inner_radius) {
                        cost = nav2_costmap_2d::LETHAL_OBSTACLE-1;
                    } else {
                        cost = static_cast<unsigned char>(Gaussian_(distance - inner_radius, amplitude, sigma));
                    }

                    // Set the cost only if it's greater than the current cost at the cell
                    if (cost > master_grid.getCost(cell_x, cell_y)) {
                        master_grid.setCost(cell_x, cell_y, cost);
                    }
                }
            }
        }
        }
        else
        {
            //TODO add  the Hidden zone if human cannot see but will see soon
            //squard Hidden Zone
            bool will_not_be_seen{true};
            for(size_t i{};i<std::min(static_cast<size_t>(50),future_path_.size());++i ) 
            {
                //future pose
                double fx{future_path_[i].position.x},fy{future_path_[i].position.y};
                unsigned int future_mx, future_my;
                if (!master_grid.worldToMap(fx, fy, future_mx, future_my)) {  // GLOBAL_COSTMAP
                    //RCLCPP_WARN(node->get_logger(), "Person is outside the costmap bounds, skipping.");
                    continue;
                }
                if(!isBlocked(master_grid,person_mx,person_my,future_mx,future_my))
                {
                    will_not_be_seen=false;
                    break;
                }
            }
            if(!will_not_be_seen)
            {
                for (int dx = -buffer_cells; dx <= buffer_cells; ++dx) 
                {
                   for (int dy = -buffer_cells; dy <= buffer_cells; ++dy)
                    {
                    unsigned int cell_x = person_mx + dx;
                    unsigned int cell_y = person_my + dy;

                        // Check if the cell is within the update window
                        if (cell_x >= min_i && cell_x < max_i && cell_y >= min_j && cell_y < max_j) {
                            // Compute Chebyshev distance to the person center
                            double distance = std::max(std::abs(dx * resolution), std::abs(dy * resolution));
                            unsigned char cost;
                            if (distance <= inner_radius) {
                                cost = nav2_costmap_2d::LETHAL_OBSTACLE-1;
                            } else {
                                cost = static_cast<unsigned char>(Gaussian_(distance - inner_radius, amplitude, sigma));
                            }

                            // Set the cost only if it's greater than the current cost at the cell
                            if (cost > master_grid.getCost(cell_x, cell_y)) {
                                master_grid.setCost(cell_x, cell_y, cost);
                            }
                        }
                    }
                }
            }
            

        }
        unsigned char current_cost{master_grid.getCost(person_mx, person_my)};
        RCLCPP_INFO(node->get_logger(),"[HUMAN_LAYER] Person:  %s in x: %lf, y: %lf, cost %u \n",guy.name.c_str(),lx,ly,current_cost);
    }

}

// Helper function 

bool HumanLayer::isBlocked(nav2_costmap_2d::Costmap2D const&master_grid, int mx, int my, int rx, int ry)
{
    int dx = std::abs(rx - mx);
    int dy = std::abs(ry - my);
    int sx = (mx < rx) ? 1 : -1;
    int sy = (my < ry) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (master_grid.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
            return true;
        }

        if (mx == rx && my == ry) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            mx += sx;
        }
        if (e2 < dx) {
            err += dx;
            my += sy;
        }
    }

    return false;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(air_navigation_layers::HumanLayer, nav2_costmap_2d::Layer)
