#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>

#include <vamp/robots/base_robot_interface.hh>
#include <vamp/robots/robot_wrapper.hh>
#include <vamp/robots/panda_grid.hh>

namespace vamp::robots
{
    /**
     * @brief Factory for creating robot instances from templates
     * 
     * This factory provides methods to create robot instances that implement
     * the RobotInterface while preserving SIMD optimizations from the template layer.
     * It maps positions to the nearest grid variant and provides access to all
     * available grid variants.
     */
    class RobotFactory
    {
    public:
        /**
         * @brief Create a Panda robot from position (maps to nearest grid variant)
         * @param x X coordinate in meters
         * @param y Y coordinate in meters  
         * @param z Z coordinate in meters
         * @return Unique pointer to RobotInterface
         * @throws std::runtime_error if position is not supported in grid
         */
        static std::unique_ptr<RobotInterface> create_panda(float x, float y, float z)
        {
            // Round to nearest grid position (convert to cm * 100)
            int grid_x = std::round(x * 100);
            int grid_y = std::round(y * 100);
            int grid_z = std::round(z * 100);
            
            // Map to template variant
            if (grid_x == 0 && grid_y == 0 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_0_0>>();
            if (grid_x == 0 && grid_y == 100 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_0_1>>();
            if (grid_x == 0 && grid_y == 200 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_0_2>>();
            if (grid_x == 100 && grid_y == 0 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_1_0>>();
            if (grid_x == 100 && grid_y == 100 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_1_1>>();
            if (grid_x == 100 && grid_y == 200 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_1_2>>();
            if (grid_x == 200 && grid_y == 0 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_2_0>>();
            if (grid_x == 200 && grid_y == 100 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_2_1>>();
            if (grid_x == 200 && grid_y == 200 && grid_z == 5) 
                return std::make_unique<RobotWrapper<Panda_2_2>>();
            
            throw std::runtime_error("Position (" + std::to_string(x) + ", " + 
                                   std::to_string(y) + ", " + std::to_string(z) + 
                                   ") not supported in grid. Available positions: " +
                                   get_available_positions_string());
        }

        /**
         * @brief Create a Panda robot from grid variant name
         * @param variant Grid variant name (e.g., "panda_2_2")
         * @return Unique pointer to RobotInterface
         * @throws std::runtime_error if variant name is unknown
         */
        static std::unique_ptr<RobotInterface> create_panda_grid(const std::string& variant)
        {
            if (variant == "panda_0_0") return std::make_unique<RobotWrapper<Panda_0_0>>();
            if (variant == "panda_0_1") return std::make_unique<RobotWrapper<Panda_0_1>>();
            if (variant == "panda_0_2") return std::make_unique<RobotWrapper<Panda_0_2>>();
            if (variant == "panda_1_0") return std::make_unique<RobotWrapper<Panda_1_0>>();
            if (variant == "panda_1_1") return std::make_unique<RobotWrapper<Panda_1_1>>();
            if (variant == "panda_1_2") return std::make_unique<RobotWrapper<Panda_1_2>>();
            if (variant == "panda_2_0") return std::make_unique<RobotWrapper<Panda_2_0>>();
            if (variant == "panda_2_1") return std::make_unique<RobotWrapper<Panda_2_1>>();
            if (variant == "panda_2_2") return std::make_unique<RobotWrapper<Panda_2_2>>();
            
            throw std::runtime_error("Unknown grid variant: " + variant + 
                                   ". Available variants: " + get_available_variants_string());
        }

        /**
         * @brief Get all available grid variant names
         * @return Vector of variant names
         */
        static std::vector<std::string> get_available_variants()
        {
            return {"panda_0_0", "panda_0_1", "panda_0_2", 
                    "panda_1_0", "panda_1_1", "panda_1_2",
                    "panda_2_0", "panda_2_1", "panda_2_2"};
        }

        /**
         * @brief Get all available grid positions
         * @return Vector of [x, y, z] position arrays
         */
        static std::vector<std::array<float, 3>> get_available_positions()
        {
            return {{0.0f, 0.0f, 0.05f}, {0.0f, 1.0f, 0.05f}, {0.0f, 2.0f, 0.05f},
                    {1.0f, 0.0f, 0.05f}, {1.0f, 1.0f, 0.05f}, {1.0f, 2.0f, 0.05f},
                    {2.0f, 0.0f, 0.05f}, {2.0f, 1.0f, 0.05f}, {2.0f, 2.0f, 0.05f}};
        }

        /**
         * @brief Get the nearest grid position to a given position
         * @param x X coordinate in meters
         * @param y Y coordinate in meters
         * @param z Z coordinate in meters
         * @return Array of [x, y, z] nearest grid position
         */
        static std::array<float, 3> get_nearest_grid_position(float x, float y, float z)
        {
            // Round to nearest grid position
            int grid_x = std::round(x * 100);
            int grid_y = std::round(y * 100);
            int grid_z = std::round(z * 100);
            
            // Clamp to valid grid range
            grid_x = std::max(0, std::min(200, grid_x));
            grid_y = std::max(0, std::min(200, grid_y));
            grid_z = std::max(5, std::min(5, grid_z));  // Z is fixed at 0.05m
            
            return {grid_x / 100.0f, grid_y / 100.0f, grid_z / 100.0f};
        }

        /**
         * @brief Check if a position is exactly on a grid point
         * @param x X coordinate in meters
         * @param y Y coordinate in meters
         * @param z Z coordinate in meters
         * @return True if position is on grid
         */
        static bool is_on_grid(float x, float y, float z)
        {
            int grid_x = std::round(x * 100);
            int grid_y = std::round(y * 100);
            int grid_z = std::round(z * 100);
            
            // Check if rounded position matches input (within small tolerance)
            const float tolerance = 0.001f;  // 1mm tolerance
            return std::abs(x * 100 - grid_x) < tolerance &&
                   std::abs(y * 100 - grid_y) < tolerance &&
                   std::abs(z * 100 - grid_z) < tolerance &&
                   grid_x >= 0 && grid_x <= 200 && grid_x % 100 == 0 &&
                   grid_y >= 0 && grid_y <= 200 && grid_y % 100 == 0 &&
                   grid_z == 5;
        }

    private:
        /**
         * @brief Get available variants as a comma-separated string
         * @return String representation of available variants
         */
        static std::string get_available_variants_string()
        {
            auto variants = get_available_variants();
            std::string result;
            for (size_t i = 0; i < variants.size(); ++i) {
                if (i > 0) result += ", ";
                result += variants[i];
            }
            return result;
        }

        /**
         * @brief Get available positions as a string
         * @return String representation of available positions
         */
        static std::string get_available_positions_string()
        {
            auto positions = get_available_positions();
            std::string result;
            for (size_t i = 0; i < positions.size(); ++i) {
                if (i > 0) result += ", ";
                result += "(" + std::to_string(positions[i][0]) + ", " +
                         std::to_string(positions[i][1]) + ", " +
                         std::to_string(positions[i][2]) + ")";
            }
            return result;
        }
    };

}  // namespace vamp::robots 