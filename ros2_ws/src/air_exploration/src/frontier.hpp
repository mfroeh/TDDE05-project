#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

struct Map
{
    Map(OccupancyGrid::SharedPtr const grid) : width{grid->info.width},
                                               height{grid->info.height},
                                               size{grid->info.width * grid->info.height},
                                               origin{grid->info.origin},
                                               resolution{grid->info.resolution},
                                               data{grid->data} {}

    size_t width, height, size;
    Pose origin;
    float resolution;

    int8_t const &operator[](size_t index) const
    {
        return data[index];
    }

private:
    std::vector<int8_t> data;
};

struct Frontier
{
    Frontier(std::vector<unsigned> const &map_points, Map const &map) : map_points{map_points}, centroid{}, points{}
    {
        for (auto &mp : map_points)
        {
            Point p;
            p.x = ((mp % map.width) + (map.origin.position.x / map.resolution)) * map.resolution;
            p.y = ((mp / map.width) + (map.origin.position.y / map.resolution)) * map.resolution;
            p.z = 0;
            points.push_back(p);

            centroid.x += p.x;
            centroid.y += p.y;
        }
        centroid.x /= map_points.size();
        centroid.y /= map_points.size();
    }

    Point centroid;
    std::vector<Point> points;

public:
    std::vector<unsigned> map_points;
};

/// @brief Wavefront frontier detection algorithm
/// @param map The occupancy map to search (starting from map.origin)
/// @param minsize The minimum size of a frontier to be considered
/// @return A vector of frontiers
std::vector<Frontier> WFD(Map const &map, unsigned minsize = 10);