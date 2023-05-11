#include <air_navigation_layers//proxemic_layer.hpp>
#include <math.h>
#include <angles/angles.h>
#include <algorithm>
#include <list>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"

#include "air_interfaces/msg/people.hpp"
#include "air_interfaces/msg/person.hpp"

using air_interfaces::msg::Person;
using air_interfaces::msg::People;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace air_navigation_layers
{
class PassingLayer : public ProxemicLayer
{
public:
  PassingLayer() : ProxemicLayer()
  {}

  virtual void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                            double* max_x, double* max_y)
  {
    std::string global_frame = layered_costmap_->getGlobalFrameID();
    transformed_people_.clear();

    for (unsigned int i = 0; i < people_list_.people.size(); i++)
    {
      Person& person = people_list_.people[i];
      Person tpt;
      geometry_msgs::msg::PointStamped pt, opt;

      try
      {
        pt.point.x = person.position.x;
        pt.point.y = person.position.y;
        pt.point.z = person.position.z;
        pt.header.frame_id = people_list_.header.frame_id;
        pt.header.stamp = people_list_.header.stamp;
        tf_->transform(pt, opt, global_frame);
        tpt.position.x = opt.point.x;
        tpt.position.y = opt.point.y;
        tpt.position.z = opt.point.z;

        pt.point.x += person.velocity.x;
        pt.point.y += person.velocity.y;
        pt.point.z += person.velocity.z;
        tf_->transform(pt, opt, global_frame);

        tpt.velocity.x = tpt.position.x - opt.point.x;
        tpt.velocity.y = tpt.position.y - opt.point.y;
        tpt.velocity.z = tpt.position.z - opt.point.z;

        transformed_people_.push_back(tpt);

        double mag = sqrt(pow(tpt.velocity.x, 2) + pow(person.velocity.y, 2));
        double factor = 1.0 + mag * factor_;
        double point = get_radius(cutoff_, amplitude_, covar_ * factor);

        *min_x = std::min(*min_x, tpt.position.x - point);
        *min_y = std::min(*min_y, tpt.position.y - point);
        *max_x = std::max(*max_x, tpt.position.x + point);
        *max_y = std::max(*max_y, tpt.position.y + point);
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
  }

  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    //boost::recursive_mutex::scoped_lock lock(lock_);
    if (!enabled_) return;

    if (people_list_.people.size() == 0)
      return;
    if (cutoff_ >= amplitude_)
      return;

    std::list<Person>::iterator p_it;
    //nav2_costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    double res = master_grid.getResolution();

    for (p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
    {
      Person person = *p_it;
      double angle = atan2(person.velocity.y, person.velocity.x) + 1.51;
      double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
      double factor = 1.0 + mag * factor_;
      double base = get_radius(cutoff_, amplitude_, covar_);
      double point = get_radius(cutoff_, amplitude_, covar_ * factor);

      unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                   height = std::max(1, static_cast<int>((base + point) / res));

      double cx = person.position.x, cy = person.position.y;

      double ox, oy;
      if (sin(angle) > 0)
        oy = cy - base;
      else
        oy = cy + (point - base) * sin(angle) - base;

      if (cos(angle) >= 0)
        ox = cx - base;
      else
        ox = cx + (point - base) * cos(angle) - base;


      int dx, dy;
      master_grid.worldToMapNoBounds(ox, oy, dx, dy);

      int start_x = 0, start_y = 0, end_x = width, end_y = height;
      if (dx < 0)
        start_x = -dx;
      else if (dx + width > master_grid.getSizeInCellsX())
        end_x = std::max(0, static_cast<int>(master_grid.getSizeInCellsX()) - dx);

      if (static_cast<int>(start_x + dx) < min_i)
        start_x = min_i - dx;
      if (static_cast<int>(end_x + dx) > max_i)
        end_x = max_i - dx;

      if (dy < 0)
        start_y = -dy;
      else if (dy + height > master_grid.getSizeInCellsY())
        end_y = std::max(0, static_cast<int>(master_grid.getSizeInCellsY()) - dy);

      if (static_cast<int>(start_y + dy) < min_j)
        start_y = min_j - dy;
      if (static_cast<int>(end_y + dy) > max_j)
        end_y = max_j - dy;

      double bx = ox + res / 2,
             by = oy + res / 2;
      for (int i = start_x; i < end_x; i++)
      {
        for (int j = start_y; j < end_y; j++)
        {
          unsigned char old_cost = master_grid.getCost(i + dx, j + dy);
          if (old_cost == nav2_costmap_2d::NO_INFORMATION)
            continue;

          double x = bx + i * res, y = by + j * res;
          double ma = atan2(y - cy, x - cx);
          double diff = angles::shortest_angular_distance(angle, ma);
          double a;
          if (fabs(diff) < M_PI / 2)
            a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);
          else
            continue;

          if (a < cutoff_)
            continue;
          unsigned char cvalue = (unsigned char) a;
          master_grid.setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        }
      }
    }
  }
  virtual void reset()
  {
    return;
  }
};
};  // namespace social_navigation_layers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(air_navigation_layers::PassingLayer, nav2_costmap_2d::Layer)