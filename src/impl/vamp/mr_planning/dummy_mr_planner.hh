#pragma once

#include "mr_planner_base.hh"

namespace vamp::mr_planning
{
    /**
     * @brief Simple multi-robot planner that ignores inter-robot collisions
     * 
     * This planner provides a baseline implementation that treats each robot
     * independently, solving individual planning problems without considering
     * inter-robot collisions. This serves as a comparison baseline and testing
     * framework for more sophisticated multi-robot algorithms.
     * 
     * @tparam RobotType The robot type (e.g., Panda_0_0)
     * @tparam rake SIMD vector width
     * @tparam resolution Collision checking resolution
     */
    template<typename RobotType, std::size_t rake, std::size_t resolution>
    class DummyMRPlanner : public MRPlannerBase<RobotType, rake, resolution>
    {
    public:
        using Base = MRPlannerBase<RobotType, rake, resolution>;
        using Configuration = typename Base::Configuration;
        using Environment = typename Base::Environment;
        using RNG = typename Base::RNG;

        /**
         * @brief Constructor
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @param rng Random number generator
         * @param settings Multi-robot planning settings
         */
        DummyMRPlanner(const std::vector<std::array<float, 3>>& base_positions,
                       const Environment& env,
                       std::shared_ptr<RNG> rng,
                       const MRSettings& settings = MRSettings())
            : Base(base_positions, env, rng, settings)
        {
            // Override settings to disable inter-robot collision checking for dummy planner
            auto& mutable_settings = const_cast<MRSettings&>(this->get_settings());
            mutable_settings.enable_inter_robot_collision_checking = false;
            mutable_settings.algorithm = "dummy";
        }

        /**
         * @brief Solve multi-robot planning problem
         * @param starts Start configurations for each robot
         * @param goals Goal configurations for each robot
         * @return Planning result with paths for each robot
         */
        MRPlanningResult<Base::dimension> solve(const std::vector<Configuration>& starts,
                                               const std::vector<Configuration>& goals)
        {
            return this->solve_ignoring_inter_robot_collisions(starts, goals);
        }

        /**
         * @brief Solve multi-robot planning problem using MRProblem structure
         * @param problem Multi-robot problem definition
         * @return Planning result with paths for each robot
         */
        MRPlanningResult<Base::dimension> solve(const MRProblem<Base::dimension>& problem)
        {
            if (!problem.is_valid()) {
                throw std::runtime_error("Invalid multi-robot problem");
            }

            if (problem.num_robots() != this->num_robots()) {
                throw std::runtime_error("Problem robot count does not match planner robot count");
            }

            return solve(problem.start_configurations, problem.goal_configurations);
        }

        /**
         * @brief Get planner name
         * @return Name of the planner
         */
        std::string get_name() const {
            return "DummyMRPlanner";
        }

        /**
         * @brief Get planner description
         * @return Description of the planner
         */
        std::string get_description() const {
            return "Simple multi-robot planner that ignores inter-robot collisions. "
                   "Each robot is planned independently using RRTConnect.";
        }

    protected:
        /**
         * @brief Override to add dummy-specific behavior when roadmap is built
         * @param robot_idx Index of the robot
         */
        void on_roadmap_built(std::size_t robot_idx) override {
            // For dummy planner, we could add some logging or validation here
            // For now, just call the base implementation
            Base::on_roadmap_built(robot_idx);
        }
    };

}  // namespace vamp::mr_planning 