#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

// http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
#define PRIOR 20

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

struct Map
{
    Map(OccupancyGrid::SharedPtr const grid) : width{grid->info.width},
                                               height{grid->info.height},
                                               origin{grid->info.origin},
                                               resolution{grid->info.resolution},
                                               data{grid->data} {}

    size_t width, height;
    Pose origin;
    float resolution;

    int8_t get(size_t index) const
    {
        assert(index < width * height);
        return data[index];
    }

    int8_t get(size_t y, size_t x) const
    {
        assert(y < height && x < width);
        return data[y * width + x];
    }

    size_t index_from_point(Point p) const
    {
        // Calculate distance between pos and occupancy grid origin
        double dx{p.x - origin.position.x};
        double dy{p.y - origin.position.y};

        // Convert distance to cells
        size_t cell_x{std::round(dx / resolution)};
        size_t cell_y{std::round(dy / resolution)};

        return cell_y * width + cell_x;
    }

    Point point_from_index(size_t index) const
    {
        Point p{};
        // TODO
        return p;
    }

private:
    std::vector<int8_t> data;
};

struct Cell
{
    size_t x, y;

    Cell(size_t x, size_t y) : x{x}, y{y} {}
    Cell(size_t index, Map const &map) : x{index % map.width}, y{index / map.width} {}

    size_t to_index(Map const &map) const
    {
        return y * map.width + x;
    }

    std::vector<Cell> adjacent(Map const &map) const
    {
        std::vector<Cell> adj{};
        adj.push_back({x - 1, y});
        adj.push_back({x + 1, y});
        adj.push_back({x, y - 1});
        adj.push_back({x, y + 1});
        adj.erase(std::remove_if(adj.begin(), adj.end(), [map](Cell const &c)
                                 { return !c.valid(map); }),
                  adj.end());
        return adj;
    }

    std::vector<Cell> adjacent_diag(Map const &map) const
    {
        std::vector<Cell> adj{adjacent(map)};
        adj.push_back({x - 1, y - 1});
        adj.push_back({x - 1, y + 1});
        adj.push_back({x + 1, y - 1});
        adj.push_back({x + 1, y + 1});
        adj.erase(std::remove_if(adj.begin(), adj.end(), [map](Cell const &c)
                                 { return !c.valid(map); }),
                  adj.end());
        return adj;
    }

    bool is_frontier(Map const &map) const
    {
        if (map.get(y, x) != -1)
            return false;

        std::vector<Cell> adj{adjacent_diag(map)};
        bool adj_open{false};
        for (auto &c : adj)
        {
            // Adjacent cell is occupied
            if (map.get(c.y, c.x) >= PRIOR)
                return false;

            if (map.get(c.y, c.x) < PRIOR)
                adj_open = true;
        }

        return adj_open;
    }

    bool valid(Map const &map) const
    {
        return x > 0 && y > 0 && x < map.width && y < map.height;
    }
};

struct Frontier
{
    std::vector<Cell> cells{};
};

std::vector<Frontier> WFD(Point start, Map const &map);