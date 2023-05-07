#include <queue>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
#define PRIOR 20

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;

struct Map
{
    size_t width{}, height{};
    Pose origin{};
    float resolution{};

    void update(OccupancyGrid::SharedPtr const grid)
    {
        // TODO
    }

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
    std::vector<int8_t> data{};
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

std::vector<Frontier> WFD(Point start, Map const &map)
{
    std::vector<Frontier> result{};

    enum WFDState
    {
        NONE = 0,
        MAP_OPEN_LIST,       // Points that have been enqueued by the outer BFS
        MAP_CLOSE_LIST,      // Points that have been dequeued by the outer BFS
        FRONTIER_OPEN_LIST,  // Points that have been enqueued by the inner BFS
        FRONTIER_CLOSE_LIST, // Points that have been dequeued by the inner BFS
    };

    // Get starting cell
    Cell pose{map.index_from_point(start), map};
    std::cout << "Starting cell: " << pose.x << ", " << pose.y << std::endl;

    std::vector<WFDState> states{map.height * map.width}; // The state of each cell
    std::queue<Cell> queue_m{};                           // queue used for detecting frontiers
    std::queue<Cell> queue_f{};                           // queue used for extracting a frontier from a given frontier cell
    queue_m.push(pose);
    states[pose.to_index(map)] = MAP_OPEN_LIST;

    while (!queue_m.empty())
    {
        Cell p{queue_m.front()};
        queue_m.pop();

        if (states[p.to_index(map)] == MAP_CLOSE_LIST)
            continue;

        if (p.is_frontier(map))
        {
            queue_f = {};
            Frontier new_frontier{};
            queue_f.push(p);
            states[p.to_index(map)] = FRONTIER_OPEN_LIST;

            while (!queue_f.empty())
            {
                Cell q{queue_f.front()};
                queue_f.pop();

                if ((states[q.to_index(map)] == MAP_CLOSE_LIST || states[q.to_index(map)] == FRONTIER_CLOSE_LIST))
                    continue;

                if (q.is_frontier(map))
                {
                    new_frontier.cells.push_back(q);
                    for (auto &w : q.adjacent_diag(map))
                    {
                        WFDState mark{states[w.to_index(map)]};
                        if (mark != FRONTIER_OPEN_LIST &&
                            mark != FRONTIER_CLOSE_LIST && mark != MAP_CLOSE_LIST)
                        {
                            queue_f.push(w);
                            states[w.to_index(map)] = FRONTIER_OPEN_LIST;
                        }
                    }
                }

                states[q.to_index(map)] = FRONTIER_CLOSE_LIST;
                result.push_back(new_frontier);

                // Mark all points of new frontier as MAP_CLOSE_LIST
                for (auto &c : new_frontier.cells)
                    states[c.to_index(map)] = MAP_CLOSE_LIST;

                for (auto &v : p.adjacent_diag(map))
                {
                    WFDState mark{states[v.to_index(map)]};
                    std::vector<Cell> adj{v.adjacent_diag(map)};
                    if (mark != MAP_OPEN_LIST && mark != MAP_CLOSE_LIST &&
                        std::any_of(adj.begin(), adj.end(), [map, states](Cell c)
                                    { return map.get(c.y, c.x) == 0; }))
                    {
                        queue_m.push(v);
                        states[v.to_index(map)] = MAP_OPEN_LIST;
                    }
                }

                states[p.to_index(map)] = MAP_CLOSE_LIST;
            }
        }
    }

    return result;
}