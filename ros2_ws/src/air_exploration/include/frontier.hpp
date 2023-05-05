#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <vector>
#include <queue>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

class Map
{
public:
    Map() : width{}, height{}, matrix{}, origin{}, resolution{} {}
    void update(const OccupancyGrid::SharedPtr grid);
    friend std::ostream &operator<<(std::ostream &os, Map const &map);

private:
    uint32_t width, height;
    std::vector<std::vector<int8_t>> matrix;
    Pose origin;
    float resolution;
};

class Frontier
{
public:
    Point center();
    float distance_to(Point point);

private:
    std::vector<Point> points;
};

class Algorithm
{
public:
    Algorithm() : map{}, frontiers{} {}
    Frontier pop();
    void update_map(const OccupancyGrid::SharedPtr grid);
    size_t count();

private:
    Map map;
    std::priority_queue<Frontier> frontiers;
};