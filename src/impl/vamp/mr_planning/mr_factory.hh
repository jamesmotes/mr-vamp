#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <stdexcept>

#include <vamp/collision/environment.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/base_robot_interface.hh>
#include <vamp/utils.hh>

#include "mr_planner_base.hh"
#include "dummy_mr_planner.hh"
#include "mr_settings.hh"

namespace vamp::mr_planning
{
    /**
     * @brief Factory for creating multi-robot planners
     * 
     * This factory creates the appropriate multi-robot planner based on the
     * specified algorithm name and configuration. It handles the template
     * instantiation with the correct SIMD vector width and resolution.
     */
    class MRPlannerFactory
    {
    public:
        using Environment = collision::Environment<float>;
        using RNG = vamp::rng::RNG<7>;  // Fixed dimension for Panda robots

        /**
         * @brief Create a multi-robot planner
         * @param algorithm_name Name of the algorithm to use
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @param rng Random number generator
         * @param settings Multi-robot planning settings
         * @return Unique pointer to the created planner
         */
        static std::unique_ptr<MRPlannerBase<4, 4>> create_planner(
            const std::string& algorithm_name,
            const std::vector<std::array<float, 3>>& base_positions,
            const Environment& env,
            std::shared_ptr<RNG> rng,
            const MRSettings& settings = MRSettings())
        {
            // For now, we only support the dummy planner
            // In the future, this could be extended to support other algorithms
            if (algorithm_name == "dummy" || algorithm_name == "ignore_inter_robot") {
                return std::make_unique<DummyMRPlanner<4, 4>>(
                    base_positions, env, rng, settings);
            }
            
            throw std::runtime_error("Unknown multi-robot planning algorithm: " + algorithm_name);
        }

        /**
         * @brief Get list of available algorithms
         * @return Vector of algorithm names
         */
        static std::vector<std::string> get_available_algorithms()
        {
            return {"dummy", "ignore_inter_robot"};
        }

        /**
         * @brief Check if an algorithm is available
         * @param algorithm_name Name of the algorithm
         * @return True if the algorithm is available
         */
        static bool is_algorithm_available(const std::string& algorithm_name)
        {
            auto algorithms = get_available_algorithms();
            return std::find(algorithms.begin(), algorithms.end(), algorithm_name) != algorithms.end();
        }
    };

}  // namespace vamp::mr_planning 