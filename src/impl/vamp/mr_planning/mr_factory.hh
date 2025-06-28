#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

#include <vamp/collision/environment.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/panda_grid.hh>
#include <vamp/vector.hh>

#include "mr_planner_base.hh"
#include "dummy_mr_planner.hh"
#include "mr_settings.hh"

namespace vamp::mr_planning
{
    /**
     * @brief Factory for creating multi-robot planners
     * 
     * This factory provides a unified interface for creating different types of
     * multi-robot planners, allowing easy algorithm selection and configuration.
     */
    class MRPlannerFactory
    {
    public:
        // Forward declarations for planner types
        template<typename RobotType, std::size_t rake, std::size_t resolution>
        using PlannerPtr = std::unique_ptr<MRPlannerBase<RobotType, rake, resolution>>;

        /**
         * @brief Create a multi-robot planner
         * @param algorithm Algorithm name ("dummy", "cbs", "prioritized", etc.)
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @param settings Multi-robot planning settings
         * @param rng Random number generator
         * @return Unique pointer to planner
         */
        template<typename RobotType, std::size_t rake, std::size_t resolution>
        static PlannerPtr<RobotType, rake, resolution> create_planner(
            const std::string& algorithm,
            const std::vector<std::array<float, 3>>& base_positions,
            const collision::Environment<float>& env,
            const MRSettings& settings,
            std::shared_ptr<vamp::rng::RNG<RobotType::dimension>> rng)
        {
            if (algorithm == "dummy") {
                return std::make_unique<DummyMRPlanner<RobotType, rake, resolution>>(
                    base_positions, env, rng, settings);
            }
            // Add more algorithms here as they are implemented
            // else if (algorithm == "cbs") {
            //     return std::make_unique<CBSMRPlanner<RobotType, rake, resolution>>(
            //         base_positions, env, rng, settings);
            // }
            else {
                throw std::runtime_error("Unknown multi-robot planning algorithm: " + algorithm);
            }
        }

        /**
         * @brief Create a dummy multi-robot planner (convenience method)
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @param settings Multi-robot planning settings
         * @param rng Random number generator
         * @return Unique pointer to dummy planner
         */
        template<typename RobotType, std::size_t rake, std::size_t resolution>
        static PlannerPtr<RobotType, rake, resolution> create_dummy_planner(
            const std::vector<std::array<float, 3>>& base_positions,
            const collision::Environment<float>& env,
            const MRSettings& settings = MRSettings(),
            std::shared_ptr<vamp::rng::RNG<RobotType::dimension>> rng = nullptr)
        {
            if (!rng) {
                rng = std::make_shared<vamp::rng::Halton<RobotType::dimension>>();
            }
            return create_planner<RobotType, rake, resolution>("dummy", base_positions, env, settings, rng);
        }

        /**
         * @brief Get all available algorithm names
         * @return Vector of algorithm names
         */
        static std::vector<std::string> get_available_algorithms()
        {
            return {"dummy"};  // Add more as they are implemented
        }

        /**
         * @brief Check if an algorithm is available
         * @param algorithm Algorithm name
         * @return True if algorithm is available
         */
        static bool is_algorithm_available(const std::string& algorithm)
        {
            auto algorithms = get_available_algorithms();
            return std::find(algorithms.begin(), algorithms.end(), algorithm) != algorithms.end();
        }

        /**
         * @brief Get description of an algorithm
         * @param algorithm Algorithm name
         * @return Algorithm description
         */
        static std::string get_algorithm_description(const std::string& algorithm)
        {
            if (algorithm == "dummy") {
                return "Simple multi-robot planner that ignores inter-robot collisions. "
                       "Each robot is planned independently using RRTConnect.";
            }
            // Add more descriptions as algorithms are implemented
            else {
                return "Unknown algorithm";
            }
        }

        /**
         * @brief Create a planner with default settings for Panda robots
         * @param algorithm Algorithm name
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @return Unique pointer to planner
         */
        static std::unique_ptr<MRPlannerBase<vamp::robots::Panda_0_0, 8, 32>> create_panda_planner(
            const std::string& algorithm,
            const std::vector<std::array<float, 3>>& base_positions,
            const collision::Environment<float>& env,
            const MRSettings& settings = MRSettings())
        {
            auto rng = std::make_shared<vamp::rng::Halton<7>>();
            return create_planner<vamp::robots::Panda_0_0, 8, 32>(
                algorithm, base_positions, env, settings, rng);
        }

        /**
         * @brief Create a dummy planner with default settings for Panda robots
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @return Unique pointer to dummy planner
         */
        static std::unique_ptr<MRPlannerBase<vamp::robots::Panda_0_0, 8, 32>> create_dummy_panda_planner(
            const std::vector<std::array<float, 3>>& base_positions,
            const collision::Environment<float>& env,
            const MRSettings& settings = MRSettings())
        {
            return create_panda_planner("dummy", base_positions, env, settings);
        }

    private:
        // Prevent instantiation
        MRPlannerFactory() = delete;
    };

}  // namespace vamp::mr_planning 