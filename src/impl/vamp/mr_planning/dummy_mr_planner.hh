#pragma once

#include "mr_planner_base.hh"

namespace vamp::mr_planning
{
    /**
     * @brief Dummy multi-robot planner for testing and validation
     * 
     * This planner implements a simple approach that ignores inter-robot collisions
     * and plans for each robot independently. It serves as a baseline for testing
     * the multi-robot planning framework.
     * 
     * @tparam rake SIMD vector width
     * @tparam resolution Collision checking resolution
     */
    template<std::size_t rake, std::size_t resolution>
    class DummyMRPlanner : public MRPlannerBase<rake, resolution>
    {
    public:
        using Base = MRPlannerBase<rake, resolution>;
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
        }

        /**
         * @brief Solve multi-robot planning problem
         * @param starts Start configurations for each robot
         * @param goals Goal configurations for each robot
         * @return Planning result with paths for each robot
         */
        MRPlanningResult<7> solve(const std::vector<vamp::FloatVector<7>>& starts,
                                 const std::vector<vamp::FloatVector<7>>& goals)
        {
            // For dummy planner, just use the ignore inter-robot collisions method
            return this->solve_ignoring_inter_robot_collisions(starts, goals);
        }

        /**
         * @brief Solve multi-robot planning problem using MRProblem structure
         * @param problem Multi-robot problem definition
         * @return Planning result with paths for each robot
         */
        MRPlanningResult<7> solve(const MRProblem<7>& problem)
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
            // Dummy planner doesn't need any special handling
            Base::on_roadmap_built(robot_idx);
        }
    };

}  // namespace vamp::mr_planning 