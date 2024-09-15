#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <SFML/System/Vector2.hpp>

class SpatialHashing {
private:
    static constexpr int DEFAULT_CELL_SIZE = 9;
    static constexpr float DEFAULT_DISTANCE_CHECK = 5.0f;
    int m_width;
    int m_height;
    int m_cell_size;
    float m_distance_check;
    int m_grid_width;
    int m_grid_height;
    std::vector<std::vector<uint32_t>> m_grids;

    inline int hashCoords(int x, int y) const {
        return x + y * m_grid_width;
    }

public:
    SpatialHashing(int width, int height, int cell_size = DEFAULT_CELL_SIZE, float distance_check = DEFAULT_DISTANCE_CHECK)
        : m_width(width), m_height(height), m_cell_size(cell_size), m_distance_check(distance_check) {
        m_grid_width = width / cell_size + 1;
        m_grid_height = height / cell_size + 1;
        m_grids.resize(m_grid_width * m_grid_height);
    }

    void add_object(const sf::Vector2f& pos, uint32_t idx) {
        int grid_x = std::clamp(static_cast<int>(pos.x) / m_cell_size, 0, m_grid_width - 1);
        int grid_y = std::clamp(static_cast<int>(pos.y) / m_cell_size, 0, m_grid_height - 1);
        int grid_index = hashCoords(grid_x, grid_y);
        m_grids[grid_index].push_back(idx);
    }

    void clear() {
        for (auto& cell : m_grids) {
            cell.clear();
        }
    }

    const std::vector<std::vector<uint32_t>>& getGrids() const {
        return m_grids;
    }

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    int getGridWidth() const { return m_grid_width; }
    int getGridHeight() const { return m_grid_height; }

    const std::vector<uint32_t>& getCell(int grid_x, int grid_y) const {
        int index = hashCoords(grid_x, grid_y);
        return m_grids[index];
    }

    std::vector<uint32_t> getNeighbors(const sf::Vector2f& pos) const {
        std::vector<uint32_t> neighbors;
        int grid_x = static_cast<int>(pos.x) / m_cell_size;
        int grid_y = static_cast<int>(pos.y) / m_cell_size;
        
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int nx = std::clamp(grid_x + dx, 0, m_grid_width - 1);
                int ny = std::clamp(grid_y + dy, 0, m_grid_height - 1);
                const auto& cell = getCell(nx, ny);
                neighbors.insert(neighbors.end(), cell.begin(), cell.end());
            }
        }
        return neighbors;
    }

    void optimize() {
        for (auto& cell : m_grids) {
            cell.shrink_to_fit();
        }
    }
};