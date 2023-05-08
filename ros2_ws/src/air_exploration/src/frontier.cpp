#include "frontier.hpp"
<<<<<<< HEAD

#include <queue>
#include <algorithm>

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
=======
#include <algorithm>
#include <numeric>
#include <queue>

std::ostream &operator<<(std::ostream &os, Map const &map)
{
    os << "Size: " << map.width << "x" << map.height << std::endl;
    // TODO: Somehow prints unicode characters
    for (size_t i{}; i < map.height; ++i)
    {
        for (size_t j{}; j < map.width; ++j)
            os << map.matrix.at(i).at(j) << " ";
        os << std::endl;
    }
    return os;
}

void Map::update(const OccupancyGrid::SharedPtr grid)
{
    // TODO: Maybe we can just always replace the entire map with grid
    if (height == 0 || width == 0)
    {
        height = grid->info.height;
        width = grid->info.width;
        resolution = grid->info.resolution;
        origin = grid->info.origin;
        matrix = std::vector<std::vector<int8_t>>{height, std::vector<int8_t>{}};
        for (size_t row{}; row < height; ++row)
            std::copy(&grid->data[row * width], &grid->data[(row + 1) * width], std::back_inserter(matrix[row]));
    }
    else
    {
        resolution = grid->info.resolution;
        origin = grid->info.origin;

        if (height != grid->info.height)
        {
            height = grid->info.height;
            matrix.resize(height);
        }

        if (width != grid->info.width)
        {
            width = grid->info.width;
            for (size_t row{}; row < height; ++row)
                matrix[row].resize(width);
        }

        for (size_t row{}; row < height; ++row)
            for (size_t col{}; col < width; ++col)
                matrix[row][col] = grid->data[row * width + col];
    }
}

int8_t Map::at(int x, int y) const
{
    return matrix.at(y).at(x);
}

uint32_t Map::get_width() const
{
    return width;
}

uint32_t Map::get_height() const
{
    return height;
}

std::tuple<size_t, size_t> Map::index_from_point(Point pos) const
{
}

Point Map::point_from_index(size_t x, size_t y) const
{
    return {};
}

Point Frontier::center()
{
    Point sum{std::accumulate(points.begin(), points.end(), Point{}, [](Point &p1, Point const &p2)
                              { p1.x += p2.x; p1.y += p2.y; return p1; })};
    sum.x /= points.size();
    sum.y /= points.size();
    return sum;
}

float Frontier::distance_to(Point point)
{
}

bool Frontier::operator<(Frontier const &other) const
{
    return true;
}

Frontier Algorithm::pop()
{
}


void Algorithm::update_map(const OccupancyGrid::SharedPtr grid)
{
    map.update(grid);

    // TODO
    // Frontier f{};
    // f.points.push_back(Point{map.at(map.height() - 1, map.width() - 1)});
    // frontiers.push(f);
}

size_t Algorithm::count()
{
    return frontiers.size();
}
>>>>>>> 70949e01107bdc407431d1dab316d46ab8db6e5c
