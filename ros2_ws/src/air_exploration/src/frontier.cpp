#include "frontier.hpp"

#include <queue>
#include <algorithm>
#include <omp.h>
#include <map>

// http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
#define THRESHOLD 10

void adj(unsigned p, Map const &map, unsigned buffer[8])
{
    buffer[0] = p - map.width - 1; // top left
    buffer[1] = p - map.width;     // top
    buffer[2] = p - map.width + 1; // top right
    buffer[3] = p - 1;             // left
    buffer[4] = p + 1;             // right
    buffer[5] = p + map.width - 1; // bottom left
    buffer[6] = p + map.width;     // bottom
    buffer[7] = p + map.width + 1; // bottom right
}

bool is_frontier(unsigned p, Map const &map)
{
    // Point must be unknown
    if (map[p] != -1)
        return false;

    unsigned neighbors[8];
    adj(p, map, neighbors);
    for (auto &v : neighbors)
    {
        // Not actually neighbors (>= map.size instead of < 0 because of unsigned)
        if (v >= map.size)
            continue;

        // If a neighbor is occupied
        if (map[v] > THRESHOLD)
            return false;

        // TODO: Test <= THRESHOLD?
        if (map[v] == 0)
            return true;
    }
    return false;
}

std::vector<Frontier> WFD(Map const &map, unsigned minsize)
{
    enum State : int8_t
    {
        NONE = 0,
        MAP_OPEN_LIST,       // Points that have been enqueued by the outer BFS
        MAP_CLOSE_LIST,      // Points that have been dequeued by the outer BFS
        FRONTIER_OPEN_LIST,  // Points that have been enqueued by the inner BFS
        FRONTIER_CLOSE_LIST, // Points that have been dequeued by the inner BFS
    };

    float x{map.origin.position.x / map.resolution};
    float y{map.origin.position.y / map.resolution};
    unsigned pose{(-y * map.width) - x};

    std::queue<unsigned> queue_m{};
    State states[map.size]{};
    queue_m.push(pose);
    states[pose] = MAP_OPEN_LIST;

    std::vector<Frontier> frontiers{};
    int outer{}, inner{};

    // Outer BFS to find frontier points
    while (!queue_m.empty())
    {
        outer++;
        unsigned p{queue_m.front()};
        queue_m.pop();

        // If already visited
        if (states[p] == MAP_CLOSE_LIST)
            continue;

        if (is_frontier(p, map))
        {
            std::queue<unsigned> queue_f{};
            std::vector<unsigned> points{};
            queue_f.push(p);
            states[p] = FRONTIER_OPEN_LIST;

            // Inner BFS to find adjacent frontier points
            while (!queue_f.empty())
            {
                inner++;
                unsigned q{queue_f.front()};
                queue_f.pop();

                // If already visited by outer BFS or belongs to other frontier
                if (states[q] == MAP_CLOSE_LIST || states[q] == FRONTIER_CLOSE_LIST)
                    continue;

                if (is_frontier(q, map))
                {
                    points.push_back(q);
                    unsigned adj_q[8];
                    adj(q, map, adj_q);

                    for (auto &w : adj_q)
                    {
                        // Not actually neighbors (>= map.size instead of < 0 because of unsigned)
                        if (w >= map.size)
                            continue;

                        if (states[w] != FRONTIER_OPEN_LIST && states[w] != FRONTIER_CLOSE_LIST && states[w] != MAP_CLOSE_LIST)
                        {
                            if (map[w] != 100)
                            {
                                queue_f.push(w);
                                states[w] = FRONTIER_OPEN_LIST;
                            }
                        }
                    }
                }
                states[q] = FRONTIER_CLOSE_LIST;
            }

            if (points.size() >= minsize)
                frontiers.push_back(Frontier{points, map});

            // Mark all points of new frontier as MAP_CLOSE_LIST
            for (auto &c : points)
                states[c] = MAP_CLOSE_LIST;
        }

        unsigned adj_p[8];
        adj(p, map, adj_p);
        for (auto &v : adj_p)
        {
            // Not actually neighbors (>= map.size instead of < 0 because of unsigned)
            if (v >= map.size)
                continue;

            // If not already queued, not visited by outer BFS, doesn't belong to frontier and has open space neighbors
            if (states[v] != MAP_OPEN_LIST && states[v] != MAP_CLOSE_LIST)
            {
                unsigned adj_v[8];
                adj(v, map, adj_v);
                // If v has atleast one open space neighbor
                for (auto &w : adj_v)
                {
                    if (w >= map.size)
                        continue;

                    if (map[w] <= THRESHOLD && map[w] >= 0)
                    {
                        queue_m.push(v);
                        states[v] = MAP_OPEN_LIST;
                        break;
                    }
                }
            }
        }
        states[p] = MAP_CLOSE_LIST;
    }

    return frontiers;
}

std::vector<Frontier> frontier_search(Map const &map, unsigned minsize) {}

std::vector<Frontier> parallel_search(Map const &map, unsigned minsize)
{
    std::map<unsigned, bool> frontier_points{};
#pragma omp thread_num(4) parallel for schedule(dynamic, 5000)
    for (unsigned i = 0; i < map.size; ++i)
    {
        if (is_frontier(i, map))
#pragma omp critical
            frontier_points[i] = true;
    }

    std::vector<Frontier> frontiers{};
    for (auto &pair : frontier_points)
    {
        unsigned p{pair.first};
        if (!pair.second)
            continue;

        std::queue<unsigned> queue_f{};
        std::vector<unsigned> points{};
        queue_f.push(p);

        // Inner BFS to find adjacent frontier points
        while (!queue_f.empty())
        {
            unsigned q{queue_f.front()};
            queue_f.pop();

            if (!frontier_points[q])
                continue;

            points.push_back(q);
            unsigned adj_q[8];
            adj(q, map, adj_q);
            for (auto &w : adj_q)
            {
                if (frontier_points[w])
                {
                    queue_f.push(w);
                    frontier_points[q] = false;
                }
            }
        }

        if (points.size() >= minsize)
            frontiers.push_back(Frontier{points, map});
    }

    return frontiers;
}