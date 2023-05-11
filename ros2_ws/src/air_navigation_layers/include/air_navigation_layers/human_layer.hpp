#pragma once

#include <list>
#include <memory>

#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"

#include "air_lab_interfaces/msg/people.hpp"
#include "air_lab_interfaces/msg/person.hpp"

using air_lab_interfaces::msg::Person;
using air_lab_interfaces::msg::People;

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

protected:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool first_time_;
  boost::recursive_mutex lock_;
  void peopleCallback(const People::SharedPtr people);
  rclcpp::Subscription<People>::SharedPtr people_sub_; 
  People people_list_;
  std::list<Person> transformed_people_;
  
};

}  // namespace air_navigation_layers