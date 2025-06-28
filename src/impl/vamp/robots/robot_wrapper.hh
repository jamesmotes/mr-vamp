#pragma once

#include <vamp/robots/base_robot_interface.hh>
#include <vamp/bindings/common.hh>
#include <vamp/planning/phs.hh>

namespace vamp::robots
{
    /**
     * @brief Template wrapper that preserves SIMD optimizations while providing OO interface
     * 
     * This wrapper implements the RobotInterface by delegating all calls to the
     * templated robot type. This preserves all SIMD optimizations while enabling
     * multi-robot algorithms that require runtime polymorphism.
     * 
     * @tparam RobotType The templated robot type (e.g., Panda_2_2)
     */
    template<typename RobotType>
    class RobotWrapper : public RobotInterface
    {
    public:
        // Core robot functionality
        std::vector<collision::Sphere<float>> fk(const Configuration& config) override
        {
            return vamp::binding::Helper<RobotType>::fk(config);
        }

        bool validate(const Configuration& config, const collision::Environment<float>& env) override
        {
            return vamp::binding::Helper<RobotType>::validate(config, env);
        }

        std::array<float, 3> get_base_position() const override
        {
            // Convert from integer template parameters (cm * 100) to float
            return {RobotType::base_x / 100.0f, RobotType::base_y / 100.0f, RobotType::base_z / 100.0f};
        }

        std::string get_name() const override
        {
            return RobotType::name;
        }

        // Robot properties
        int get_dimension() const override
        {
            return RobotType::dimension;
        }

        int get_n_spheres() const override
        {
            return RobotType::n_spheres;
        }

        float get_resolution() const override
        {
            return RobotType::resolution;
        }

        float get_space_measure() const override
        {
            return RobotType::space_measure();
        }

        // Planning methods
        PlanningResult rrtc(const Configuration& start, const Configuration& goal,
                           const collision::Environment<float>& env,
                           const planning::RRTCSettings& settings,
                           std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::rrtc_single(start, goal, env, settings, rng);
        }

        PlanningResult rrtc(const Configuration& start, const std::vector<Configuration>& goals,
                           const collision::Environment<float>& env,
                           const planning::RRTCSettings& settings,
                           std::shared_ptr<RNG> rng) override
        {
            // Convert vector of Configuration to vector of ConfigurationArray
            std::vector<typename RobotType::ConfigurationArray> goal_arrays;
            goal_arrays.reserve(goals.size());
            for (const auto& goal : goals) {
                goal_arrays.push_back(goal.to_array());
            }
            return vamp::binding::Helper<RobotType>::rrtc(start.to_array(), goal_arrays, env, settings, rng);
        }

        PlanningResult prm(const Configuration& start, const Configuration& goal,
                          const collision::Environment<float>& env,
                          const planning::RoadmapSettings<planning::PRMStarNeighborParams>& settings,
                          std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::prm_single(start, goal, env, settings, rng);
        }

        PlanningResult prm(const Configuration& start, const std::vector<Configuration>& goals,
                          const collision::Environment<float>& env,
                          const planning::RoadmapSettings<planning::PRMStarNeighborParams>& settings,
                          std::shared_ptr<RNG> rng) override
        {
            // Convert vector of Configuration to vector of ConfigurationArray
            std::vector<typename RobotType::ConfigurationArray> goal_arrays;
            goal_arrays.reserve(goals.size());
            for (const auto& goal : goals) {
                goal_arrays.push_back(goal.to_array());
            }
            return vamp::binding::Helper<RobotType>::prm(start.to_array(), goal_arrays, env, settings, rng);
        }

        PlanningResult fcit(const Configuration& start, const Configuration& goal,
                           const collision::Environment<float>& env,
                           const planning::RoadmapSettings<planning::FCITStarNeighborParams>& settings,
                           std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::fcit(start, goal, env, settings, rng);
        }

        PlanningResult fcit(const Configuration& start, const std::vector<Configuration>& goals,
                           const collision::Environment<float>& env,
                           const planning::RoadmapSettings<planning::FCITStarNeighborParams>& settings,
                           std::shared_ptr<RNG> rng) override
        {
            // Convert vector of Configuration to vector of ConfigurationArray
            std::vector<typename RobotType::ConfigurationArray> goal_arrays;
            goal_arrays.reserve(goals.size());
            for (const auto& goal : goals) {
                goal_arrays.push_back(goal.to_array());
            }
            return vamp::binding::Helper<RobotType>::fcit_multi_goal(start.to_array(), goal_arrays, env, settings, rng);
        }

        PlanningResult aorrtc(const Configuration& start, const Configuration& goal,
                             const collision::Environment<float>& env,
                             const planning::AORRTCSettings& settings,
                             std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::aorrtc(start, goal, env, settings, rng);
        }

        PlanningResult aorrtc(const Configuration& start, const std::vector<Configuration>& goals,
                             const collision::Environment<float>& env,
                             const planning::AORRTCSettings& settings,
                             std::shared_ptr<RNG> rng) override
        {
            // Convert vector of Configuration to vector of ConfigurationArray
            std::vector<typename RobotType::ConfigurationArray> goal_arrays;
            goal_arrays.reserve(goals.size());
            for (const auto& goal : goals) {
                goal_arrays.push_back(goal.to_array());
            }
            return vamp::binding::Helper<RobotType>::aorrtc_multi_goal(start.to_array(), goal_arrays, env, settings, rng);
        }

        Roadmap roadmap(const Configuration& start, const Configuration& goal,
                       const collision::Environment<float>& env,
                       const planning::RoadmapSettings<planning::PRMStarNeighborParams>& settings,
                       std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::roadmap(start, goal, env, settings, rng);
        }

        PlanningResult simplify(const Path& path,
                               const collision::Environment<float>& env,
                               const planning::SimplifySettings& settings,
                               std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::simplify(path, env, settings, rng);
        }

        std::vector<std::vector<std::string>> sphere_validity(const Configuration& config,
                                                             const collision::Environment<float>& env) override
        {
            return vamp::binding::Helper<RobotType>::sphere_validate(config, env);
        }

        std::pair<std::array<float, 3>, std::array<float, 4>> eefk(const Configuration& config) override
        {
            return vamp::binding::Helper<RobotType>::eefk(config);
        }

        std::vector<collision::Point> filter_from_pointcloud(const std::vector<collision::Point>& pointcloud,
                                                            const Configuration& config,
                                                            const collision::Environment<float>& env,
                                                            float point_radius) override
        {
            return vamp::binding::Helper<RobotType>::filter_self_from_pointcloud(pointcloud, config, env, point_radius);
        }

        float distance(const Configuration& a, const Configuration& b) override
        {
            return a.distance(b);
        }

        // RNG factory methods
        std::shared_ptr<RNG> halton() override
        {
            return vamp::binding::Helper<RobotType>::halton();
        }

        std::shared_ptr<RNG> phs_sampler(const ProlateHyperspheroid& phs, std::shared_ptr<RNG> rng) override
        {
            return vamp::binding::Helper<RobotType>::phs_sampler(phs, rng);
        }

#if defined(__x86_64__)
        std::shared_ptr<RNG> xorshift() override
        {
            return vamp::binding::Helper<RobotType>::xorshift();
        }
#endif
    };

}  // namespace vamp::robots 