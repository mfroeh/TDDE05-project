#include "map.hpp"
#include <algorithm>

std::ostream &operator<<(std::ostream &os, Map const &map)
{
    os << "Size: " << map.width << "x" << map.height << std::endl;
    // TODO: Somehow prints unicode characters
    for (size_t i = 0; i < map.height; ++i)
    {
        for (size_t j = 0; j < map.width; ++j)
            os << map.matrix[i][j];
        os << std::endl;
    }
    return os;
}

void Map::update(const OccupancyGrid::SharedPtr grid)
{
    std::cout << height << std::endl;
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

    assert(height == grid->info.height);
    assert(width == grid->info.width);

    // Update the grid
}