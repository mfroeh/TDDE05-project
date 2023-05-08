#include "frontier.hpp"
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
