#pragma once

#include <vector>
#include <string>
#include <array>
#include <vamp/collision/environment.hh>
#include <vamp/planning/plan.hh>
#include <vamp/vector.hh>

namespace vamp::mr_planning
{
    /**
     * @brief Problem definition for multi-robot planning scenarios
     * 
     * This structure contains all the information needed to define a multi-robot
     * planning problem, including robot configurations, base positions, and environment.
     * 
     * @tparam dim Configuration space dimension
     */
    template<std::size_t dim>
    struct MRProblem
    {
        // Robot configurations
        std::vector<FloatVector<dim>> start_configurations;
        std::vector<FloatVector<dim>> goal_configurations;
        std::vector<std::array<float, 3>> base_positions;
        
        // Environment (shared by all robots)
        collision::Environment<float> environment;
        
        // Problem metadata
        std::string problem_name;
        std::vector<std::string> robot_names;
        
        /**
         * @brief Validate that the problem is well-formed
         * @return True if all vectors have matching sizes and are non-empty
         */
        bool is_valid() const {
            return start_configurations.size() == goal_configurations.size() &&
                   start_configurations.size() == base_positions.size() &&
                   start_configurations.size() == robot_names.size() &&
                   !start_configurations.empty();
        }
        
        /**
         * @brief Get the number of robots in the problem
         * @return Number of robots
         */
        std::size_t num_robots() const { return start_configurations.size(); }
        
        /**
         * @brief Add a robot to the problem
         * @param start Start configuration
         * @param goal Goal configuration
         * @param base_pos Base position (x, y, z)
         * @param name Robot name (optional)
         */
        void add_robot(const FloatVector<dim>& start, 
                      const FloatVector<dim>& goal,
                      const std::array<float, 3>& base_pos,
                      const std::string& name = "") {
            start_configurations.push_back(start);
            goal_configurations.push_back(goal);
            base_positions.push_back(base_pos);
            robot_names.push_back(name.empty() ? "robot_" + std::to_string(num_robots()) : name);
        }
        
        /**
         * @brief Clear all robots from the problem
         */
        void clear() {
            start_configurations.clear();
            goal_configurations.clear();
            base_positions.clear();
            robot_names.clear();
        }
    };

    /**
     * @brief Result of a multi-robot planning query
     * 
     * @tparam dim Configuration space dimension
     */
    template<std::size_t dim>
    struct MRPlanningResult
    {
        std::vector<planning::Path<dim>> robot_paths;
        float total_cost;
        std::size_t nanoseconds;
        std::size_t iterations;
        bool success;
        std::string algorithm_name;
        
        /**
         * @brief Default constructor
         */
        MRPlanningResult() 
            : total_cost(0.0f), nanoseconds(0), iterations(0), success(false), algorithm_name("unknown") {}
        
        /**
         * @brief Validate that the result is well-formed
         * @return True if successful and all paths are valid
         */
        bool is_valid() const {
            return success && !robot_paths.empty() && 
                   std::all_of(robot_paths.begin(), robot_paths.end(),
                              [](const auto& path) { return !path.empty(); });
        }
        
        /**
         * @brief Get the number of robot paths
         * @return Number of paths
         */
        std::size_t num_robots() const { return robot_paths.size(); }
        
        /**
         * @brief Calculate total cost across all robot paths
         * @return Sum of all path costs
         */
        float calculate_total_cost() {
            total_cost = 0.0f;
            for (const auto& path : robot_paths) {
                total_cost += path.cost();
            }
            return total_cost;
        }
    };

}  // namespace vamp::mr_planning 