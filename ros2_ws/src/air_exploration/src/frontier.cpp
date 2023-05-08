#include "frontier.hpp"

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