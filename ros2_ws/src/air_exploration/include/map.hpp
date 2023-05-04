#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <iostream>
#include <vector>

using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

struct Map
{
public:
    Map(uint32_t width, uint32_t eight) : width{width}, height{height}, matrix{}, origin{}, resolution{} {};
    void update(const OccupancyGrid::SharedPtr grid);
    friend std::ostream &operator<<(std::ostream &os, Map const &map);

private:
    uint32_t width, height;
    float resolution;
    Pose origin;
    std::vector<std::vector<int8_t>> matrix;
};