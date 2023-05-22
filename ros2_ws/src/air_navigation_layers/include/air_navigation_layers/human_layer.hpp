#pragma once

#include <list>
#include <vector>
#include <memory>
#include <cmath>

#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include "nav_msgs/msg/path.hpp"

#include "air_interfaces/msg/people.hpp"
#include "air_interfaces/msg/person.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using air_interfaces::msg::Person;
using air_interfaces::msg::People;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Path;



namespace air_navigation_layers
{

class HumanLayer : public nav2_costmap_2d::Layer
{
public:
  HumanLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double /* robot_x */, double /* robot_y */, double /* robot_yaw */, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  //virtual void onFootprintChanged();

  //virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  virtual bool isClearable() {return false;}

  //TODO: Build a funciton to check if human and robot are blocked by wall -> wall is only one has LETHAL_OBSTACLE
  /// @brief check if human and robot are blocked by wall
  /// @param master_grid The const reference of costmap
  /// @param mx The x of human in map frame
  /// @param my The y of human in map frame
  /// @param rx The x of robotic in map frame
  /// @param ry The y of robotic in map frame
  /// @return True if there is a wall, false otherwise
  virtual bool isBlocked(const nav2_costmap_2d::Costmap2D &master_grid, int mx, int my, int rx, int ry);

  //TODO: Build a funciton to check if human will see robot soon
  /// @brief check if human will see robot soon according to the plan
  /// @param master_grid The const reference of costmap 
  /// @param plan The const reference of plan
  /// @return Ture if human can see robot soon, false other wise

protected:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double robot_x_, robot_y_;
  bool first_time_,first_time_costmap_;
  boost::recursive_mutex lock_;
  void peopleCallback(const People::SharedPtr people);
  void planCallback(const Path::SharedPtr path);
  rclcpp::Subscription<People>::SharedPtr people_sub_; 
  rclcpp::Subscription<Path>::SharedPtr path_sub_; 
  People people_list_;
  Path path_list_;
  std::list<Person> transformed_people_;
  nav2_costmap_2d::Costmap2D init_map;
  std::vector<Pose> future_path_;
};

}  // namespace air_navigation_layers
