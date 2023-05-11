#pragma once

#include "air_navigation_layers/human_layer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
//#include "air_navigation_layers/ProxemicLayerConfig.hpp"

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace air_navigation_layers
{
class ProxemicLayer : public HumanLayer
{
public:
  ProxemicLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double /* robot_x */, double /* robot_y */, double /* robot_yaw */, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  //virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }
  //virtual void onFootprintChanged();
  virtual bool isClearable() {return false;}
protected:

  double cutoff_, amplitude_, covar_, factor_;
  //dynamic_reconfigure::Server<ProxemicLayerConfig>* server_;
  //dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_;
};
}  // namespace air_navigation_layers