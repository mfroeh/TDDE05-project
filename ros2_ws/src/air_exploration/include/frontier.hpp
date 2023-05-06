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
    int8_t at(int x, int y) const;
    uint32_t get_width() const;
    uint32_t get_height() const;
    Point point_from_index(size_t x, size_t y) const;
    std::tuple<size_t, size_t> index_from_point(Point pos) const;

    friend std::ostream &operator<<(std::ostream &os, Map const &map);

private:
    uint32_t width, height;
    std::vector<std::vector<int8_t>> matrix;
    Pose origin;
    float resolution;
};

struct Frontier
{
    std::vector<Point> points;
    Point center();
    float distance_to(Point point);
    bool operator<(Frontier const &other) const;
};

class Algorithm
{
public:
    Algorithm() : map{}, frontiers{} {}
    Frontier pop();
    void update_map(const OccupancyGrid::SharedPtr grid);
    size_t count();
    std::vector<Frontier> compute_frontiers(Point start);

public:
    Map map;
    std::priority_queue<Frontier> frontiers;
};