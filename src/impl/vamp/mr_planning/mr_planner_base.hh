#pragma once

#include <memory>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <future>

#include <vamp/collision/environment.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/utils.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/panda_grid.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

#include "mr_problem.hh"
#include "mr_settings.hh"

namespace vamp::mr_planning
{
    /**
     * @brief Base class for multi-robot planning algorithms
     * 
     * This class provides the foundation for all multi-robot planners, including
     * roadmap construction, robot management, and common utility functions.
     * 
     * @tparam RobotType The robot type (e.g., Panda_0_0)
     * @tparam rake SIMD vector width
     * @tparam resolution Collision checking resolution
     */
    template<typename RobotType, std::size_t rake, std::size_t resolution>
    class MRPlannerBase
    {
    public:
        using Configuration = typename RobotType::Configuration;
        static constexpr auto dimension = RobotType::dimension;
        using RNG = typename vamp::rng::RNG<RobotType::dimension>;
        using Environment = collision::Environment<float>;

    protected:
        /**
         * @brief Internal structure for managing robot roadmaps
         */
        struct RobotRoadmap {
            std::unique_ptr<planning::Roadmap<dimension>> roadmap;
            std::vector<planning::RoadmapNode> nodes;
            std::vector<planning::utils::ConnectedComponent> components;
            std::unique_ptr<planning::NN<dimension>> nn_structure;
            std::unique_ptr<float[]> states_buffer;
            std::size_t start_index;
            std::size_t goal_index;
            bool is_built = false;
            
            RobotRoadmap() : start_index(0), goal_index(1) {}
        };

        /**
         * @brief Internal structure for managing robot instances
         */
        struct RobotInstance {
            RobotRoadmap roadmap;
            Configuration start_config;
            Configuration goal_config;
            std::array<float, 3> base_position;
            std::string robot_id;
            
            RobotInstance() = default;
        };

    private:
        std::vector<RobotInstance> robots_;
        Environment environment_;
        std::shared_ptr<RNG> rng_;
        MRSettings settings_;
        
        // Timing information
        std::size_t roadmap_build_time_ns_ = 0;
        std::size_t total_planning_time_ns_ = 0;

        // Convert environment to SIMD format for planning algorithms
        using SIMDEnvironment = collision::Environment<FloatVector<rake>>;
        
        SIMDEnvironment get_simd_environment() const {
            return SIMDEnvironment(environment_);
        }

    public:
        /**
         * @brief Constructor
         * @param base_positions Base positions for each robot
         * @param env Environment with obstacles
         * @param rng Random number generator
         * @param settings Multi-robot planning settings
         */
        MRPlannerBase(const std::vector<std::array<float, 3>>& base_positions,
                      const Environment& env,
                      std::shared_ptr<RNG> rng,
                      const MRSettings& settings = MRSettings())
            : environment_(env), rng_(rng), settings_(settings)
        {
            // Create robot instances for each base position
            for (std::size_t i = 0; i < base_positions.size(); ++i) {
                RobotInstance robot_instance;
                robot_instance.base_position = base_positions[i];
                robot_instance.robot_id = "robot_" + std::to_string(i);
                
                robots_.push_back(std::move(robot_instance));
            }
        }

        /**
         * @brief Virtual destructor
         */
        virtual ~MRPlannerBase() = default;

        /**
         * @brief Delete copy constructor and assignment operator
         */
        MRPlannerBase(const MRPlannerBase&) = delete;
        MRPlannerBase& operator=(const MRPlannerBase&) = delete;

        /**
         * @brief Build roadmaps for all robots
         * @param starts Start configurations for each robot
         * @param goals Goal configurations for each robot
         */
        void build_roadmaps(const std::vector<Configuration>& starts,
                           const std::vector<Configuration>& goals)
        {
            if (starts.size() != robots_.size() || goals.size() != robots_.size()) {
                throw std::runtime_error("Number of start/goal configurations must match number of robots");
            }

            auto start_time = std::chrono::steady_clock::now();

            // Set configurations for each robot
            for (std::size_t i = 0; i < robots_.size(); ++i) {
                robots_[i].start_config = starts[i];
                robots_[i].goal_config = goals[i];
            }

            // Build roadmaps (parallel if enabled)
            if (settings_.enable_parallel_roadmap_construction && robots_.size() > 1) {
                build_roadmaps_parallel();
            } else {
                build_roadmaps_sequential();
            }

            roadmap_build_time_ns_ = vamp::utils::get_elapsed_nanoseconds(start_time);
            
            // Call virtual method for derived classes
            for (std::size_t i = 0; i < robots_.size(); ++i) {
                on_roadmap_built(i);
            }
        }

        /**
         * @brief Solve multi-robot planning problem (ignoring inter-robot collisions)
         * @param starts Start configurations for each robot
         * @param goals Goal configurations for each robot
         * @return Planning result with paths for each robot
         */
        MRPlanningResult<dimension> solve_ignoring_inter_robot_collisions(
            const std::vector<Configuration>& starts,
            const std::vector<Configuration>& goals)
        {
            auto start_time = std::chrono::steady_clock::now();
            
            MRPlanningResult<dimension> result;
            result.algorithm_name = "dummy_ignore_inter_robot";
            
            // Build roadmaps if not already built
            if (!are_roadmaps_built()) {
                build_roadmaps(starts, goals);
            }

            // Solve for each robot independently
            result.robot_paths.reserve(robots_.size());
            for (std::size_t i = 0; i < robots_.size(); ++i) {
                auto robot_result = solve_single_robot(i, starts[i], goals[i]);
                result.robot_paths.push_back(std::move(robot_result.path));
            }

            result.success = std::all_of(result.robot_paths.begin(), result.robot_paths.end(),
                                        [](const auto& path) { return !path.empty(); });
            
            result.calculate_total_cost();
            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = 1;  // Dummy algorithm uses 1 iteration

            total_planning_time_ns_ = result.nanoseconds;
            
            return result;
        }

        /**
         * @brief Check if roadmaps are built for all robots
         * @return True if all roadmaps are built
         */
        bool are_roadmaps_built() const {
            return std::all_of(robots_.begin(), robots_.end(),
                              [](const auto& robot) { return robot.roadmap.is_built; });
        }

        /**
         * @brief Get the number of robots
         * @return Number of robots
         */
        std::size_t num_robots() const { return robots_.size(); }

        /**
         * @brief Get roadmap build time
         * @return Time in nanoseconds
         */
        std::size_t get_roadmap_build_time_ns() const { return roadmap_build_time_ns_; }

        /**
         * @brief Get total planning time
         * @return Time in nanoseconds
         */
        std::size_t get_total_planning_time_ns() const { return total_planning_time_ns_; }

        /**
         * @brief Get settings
         * @return Reference to settings
         */
        const MRSettings& get_settings() const { return settings_; }

        /**
         * @brief Get environment
         * @return Reference to environment
         */
        const Environment& get_environment() const { return environment_; }

    protected:
        /**
         * @brief Virtual method called when roadmap is built for a robot
         * @param robot_idx Index of the robot
         */
        virtual void on_roadmap_built(std::size_t robot_idx) {
            // Default implementation does nothing
            (void)robot_idx;
        }

        /**
         * @brief Check inter-robot collision between two robots
         * @param robot1_idx Index of first robot (unused)
         * @param robot2_idx Index of second robot (unused)
         * @param config1 Configuration of first robot
         * @param config2 Configuration of second robot
         * @return True if robots are in collision
         */
        virtual bool check_inter_robot_collision(std::size_t robot1_idx, std::size_t robot2_idx,
                                               const Configuration& config1, 
                                               const Configuration& config2)
        {
            (void)robot1_idx;  // Suppress unused parameter warning
            (void)robot2_idx;  // Suppress unused parameter warning
            
            if (!settings_.enable_inter_robot_collision_checking) {
                return false;
            }

            // Get sphere representations for both robots using direct FK
            typename RobotType::template Spheres<rake> spheres1, spheres2;
            typename RobotType::template ConfigurationBlock<rake> block1, block2;
            
            // Convert configurations to blocks
            for (std::size_t i = 0; i < dimension; ++i) {
                block1[i] = config1[{i, 0}];
                block2[i] = config2[{i, 0}];
            }
            
            // Compute forward kinematics
            RobotType::template sphere_fk<rake>(block1, spheres1);
            RobotType::template sphere_fk<rake>(block2, spheres2);

            // Check sphere-sphere collisions with safety margin
            float safety_margin = settings_.inter_robot_safety_margin;
            for (std::size_t i = 0; i < RobotType::n_spheres; ++i) {
                for (std::size_t j = 0; j < RobotType::n_spheres; ++j) {
                    float dx = spheres1.x[{i, 0}] - spheres2.x[{j, 0}];
                    float dy = spheres1.y[{i, 0}] - spheres2.y[{j, 0}];
                    float dz = spheres1.z[{i, 0}] - spheres2.z[{j, 0}];
                    float distance_sq = dx*dx + dy*dy + dz*dz;
                    float min_distance = spheres1.r[{i, 0}] + spheres2.r[{j, 0}] + safety_margin;
                    
                    if (distance_sq < min_distance * min_distance) {
                        return true;  // Collision detected
                    }
                }
            }
            
            return false;  // No collision
        }

        /**
         * @brief Get robot instance
         * @param idx Robot index
         * @return Reference to robot instance
         */
        const RobotInstance& get_robot(std::size_t idx) const {
            if (idx >= robots_.size()) {
                throw std::out_of_range("Robot index out of range");
            }
            return robots_[idx];
        }

        /**
         * @brief Get robot instance (non-const)
         * @param idx Robot index
         * @return Reference to robot instance
         */
        RobotInstance& get_robot(std::size_t idx) {
            if (idx >= robots_.size()) {
                throw std::out_of_range("Robot index out of range");
            }
            return robots_[idx];
        }

    private:
        /**
         * @brief Build roadmaps sequentially
         */
        void build_roadmaps_sequential()
        {
            for (std::size_t i = 0; i < robots_.size(); ++i) {
                build_roadmap_for_robot(i);
            }
        }

        /**
         * @brief Build roadmaps in parallel
         */
        void build_roadmaps_parallel()
        {
            std::vector<std::future<void>> futures;
            futures.reserve(robots_.size());

            for (std::size_t i = 0; i < robots_.size(); ++i) {
                futures.push_back(std::async(std::launch::async, [this, i]() {
                    build_roadmap_for_robot(i);
                }));
            }

            // Wait for all roadmaps to be built
            for (auto& future : futures) {
                future.wait();
            }
        }

        /**
         * @brief Build roadmap for a specific robot
         * @param robot_idx Index of the robot
         */
        void build_roadmap_for_robot(std::size_t robot_idx)
        {
            auto& robot = robots_[robot_idx];
            
            // Convert environment to SIMD format for PRM
            auto simd_env = get_simd_environment();
            
            // Build roadmap using PRM
            auto roadmap_result = planning::PRM<RobotType, rake, resolution>::build_roadmap(
                robot.start_config, robot.goal_config, simd_env, settings_.roadmap_settings, rng_);
            
            // Store roadmap data
            robot.roadmap.roadmap = std::make_unique<planning::Roadmap<dimension>>(std::move(roadmap_result));
            robot.roadmap.is_built = true;
        }

        /**
         * @brief Solve single robot planning problem
         * @param robot_idx Index of the robot (unused)
         * @param start Start configuration
         * @param goal Goal configuration
         * @return Planning result
         */
        planning::PlanningResult<dimension> solve_single_robot(std::size_t robot_idx,
                                                             const Configuration& start,
                                                             const Configuration& goal)
        {
            (void)robot_idx;  // Suppress unused parameter warning
            
            // Convert environment to SIMD format for RRTC
            auto simd_env = get_simd_environment();
            
            // Use RRTConnect for single robot planning
            return planning::RRTC<RobotType, rake, resolution>::solve(
                start, goal, simd_env, settings_.rrtc_settings, rng_);
        }
    };

}  // namespace vamp::mr_planning 