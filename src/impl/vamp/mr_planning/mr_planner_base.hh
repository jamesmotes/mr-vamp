#pragma once

#include <memory>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <future>
#include <variant>

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
     * Supports heterogeneous robots with different grid positions.
     * 
     * @tparam rake SIMD vector width
     * @tparam resolution Collision checking resolution
     */
    template<std::size_t rake, std::size_t resolution>
    class MRPlannerBase
    {
    public:
        using Environment = collision::Environment<float>;
        using RNG = vamp::rng::RNG<7>;  // Fixed dimension for Panda robots

    protected:
        // Robot type variants for different grid positions
        using Panda_0_0 = vamp::robots::Panda_0_0;
        using Panda_0_1 = vamp::robots::Panda_0_1;
        using Panda_0_2 = vamp::robots::Panda_0_2;
        using Panda_1_0 = vamp::robots::Panda_1_0;
        using Panda_1_1 = vamp::robots::Panda_1_1;
        using Panda_1_2 = vamp::robots::Panda_1_2;
        using Panda_2_0 = vamp::robots::Panda_2_0;
        using Panda_2_1 = vamp::robots::Panda_2_1;
        using Panda_2_2 = vamp::robots::Panda_2_2;

        // Variant to hold different robot types
        using RobotVariant = std::variant<Panda_0_0, Panda_0_1, Panda_0_2, 
                                         Panda_1_0, Panda_1_1, Panda_1_2,
                                         Panda_2_0, Panda_2_1, Panda_2_2>;

        /**
         * @brief Internal structure for managing robot roadmaps
         */
        struct RobotRoadmap {
            std::unique_ptr<planning::Roadmap<7>> roadmap;  // Fixed dimension for Panda
            std::vector<planning::RoadmapNode> nodes;
            std::vector<planning::utils::ConnectedComponent> components;
            std::unique_ptr<planning::NN<7>> nn_structure;  // Fixed dimension for Panda
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
            RobotVariant robot_type;  // The specific robot type (grid variant)
            RobotRoadmap roadmap;
            vamp::FloatVector<7> start_config;  // Use FloatVector for SIMD compatibility
            vamp::FloatVector<7> goal_config;
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

        /**
         * @brief Get the robot type for a given base position
         * @param base_position Base position [x, y, z]
         * @return Robot variant for the position
         */
        RobotVariant get_robot_type_for_position(const std::array<float, 3>& base_position) const
        {
            // Round to nearest grid position (convert to cm * 100)
            int grid_x = std::round(base_position[0] * 100);
            int grid_y = std::round(base_position[1] * 100);
            int grid_z = std::round(base_position[2] * 100);
            
            // Map to template variant
            if (grid_x == 0 && grid_y == 0 && grid_z == 5) return Panda_0_0{};
            if (grid_x == 0 && grid_y == 100 && grid_z == 5) return Panda_0_1{};
            if (grid_x == 0 && grid_y == 200 && grid_z == 5) return Panda_0_2{};
            if (grid_x == 100 && grid_y == 0 && grid_z == 5) return Panda_1_0{};
            if (grid_x == 100 && grid_y == 100 && grid_z == 5) return Panda_1_1{};
            if (grid_x == 100 && grid_y == 200 && grid_z == 5) return Panda_1_2{};
            if (grid_x == 200 && grid_y == 0 && grid_z == 5) return Panda_2_0{};
            if (grid_x == 200 && grid_y == 100 && grid_z == 5) return Panda_2_1{};
            if (grid_x == 200 && grid_y == 200 && grid_z == 5) return Panda_2_2{};
            
            throw std::runtime_error("Position (" + std::to_string(base_position[0]) + ", " + 
                                   std::to_string(base_position[1]) + ", " + 
                                   std::to_string(base_position[2]) + 
                                   ") not supported in grid");
        }

        /**
         * @brief Get robot name from variant
         * @param robot_variant Robot variant
         * @return Robot name string
         */
        std::string get_robot_name(const RobotVariant& robot_variant) const
        {
            return std::visit([](const auto& robot) { return robot.name; }, robot_variant);
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
                
                // Get the appropriate robot type for this position
                try {
                    robot_instance.robot_type = get_robot_type_for_position(base_positions[i]);
                } catch (const std::runtime_error& e) {
                    throw std::runtime_error("Failed to create robot at position (" + 
                                           std::to_string(base_positions[i][0]) + ", " +
                                           std::to_string(base_positions[i][1]) + ", " +
                                           std::to_string(base_positions[i][2]) + "): " + e.what());
                }
                
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
        void build_roadmaps(const std::vector<vamp::FloatVector<7>>& starts,
                           const std::vector<vamp::FloatVector<7>>& goals)
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
        MRPlanningResult<7> solve_ignoring_inter_robot_collisions(
            const std::vector<vamp::FloatVector<7>>& starts,
            const std::vector<vamp::FloatVector<7>>& goals)
        {
            auto start_time = std::chrono::steady_clock::now();
            
            MRPlanningResult<7> result;
            result.algorithm_name = "dummy_ignore_inter_robot";
            
            // Build roadmaps if not already built
            if (!are_roadmaps_built()) {
                build_roadmaps(starts, goals);
            }

            // Solve for each robot independently using the correct robot type
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
         * @brief Check inter-robot collision between two robots using correct robot types
         * @param robot1_idx Index of first robot
         * @param robot2_idx Index of second robot
         * @param config1 Configuration of first robot
         * @param config2 Configuration of second robot
         * @return True if robots are in collision
         */
        virtual bool check_inter_robot_collision(std::size_t robot1_idx, std::size_t robot2_idx,
                                               const vamp::FloatVector<7>& config1, 
                                               const vamp::FloatVector<7>& config2)
        {
            if (!settings_.enable_inter_robot_collision_checking) {
                return false;
            }

            // Use std::visit to handle different robot types
            auto check_collision = [&](const auto& robot1, const auto& robot2) {
                // Get sphere representations for both robots using template-based FK
                typename std::decay_t<decltype(robot1)>::template Spheres<rake> spheres1, spheres2;
                typename std::decay_t<decltype(robot1)>::template ConfigurationBlock<rake> block1, block2;
                
                // Convert configurations to blocks
                for (std::size_t i = 0; i < 7; ++i) {
                    block1[i] = config1[{i, 0}];
                    block2[i] = config2[{i, 0}];
                }
                
                // Compute forward kinematics
                robot1.template sphere_fk<rake>(block1, spheres1);
                robot2.template sphere_fk<rake>(block2, spheres2);

                // Check sphere-sphere collisions with safety margin
                float safety_margin = settings_.inter_robot_safety_margin;
                for (std::size_t i = 0; i < robot1.n_spheres; ++i) {
                    for (std::size_t j = 0; j < robot2.n_spheres; ++j) {
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
            };

            // Check collision using the correct robot types for both robots
            return std::visit([&](const auto& robot1) {
                return std::visit([&](const auto& robot2) {
                    return check_collision(robot1, robot2);
                }, robots_[robot2_idx].robot_type);
            }, robots_[robot1_idx].robot_type);
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
         * @brief Build roadmap for a specific robot using the correct robot type
         * @param robot_idx Index of the robot
         */
        void build_roadmap_for_robot(std::size_t robot_idx)
        {
            auto& robot_instance = robots_[robot_idx];
            
            // Use std::visit to handle different robot types
            std::visit([&](const auto& robot_type) {
                // Convert environment to SIMD format for PRM
                auto simd_env = get_simd_environment();
                
                // Build roadmap using PRM with the correct robot type
                auto roadmap_result = planning::PRM<std::decay_t<decltype(robot_type)>, rake, resolution>::build_roadmap(
                    robot_instance.start_config, robot_instance.goal_config, simd_env, settings_.roadmap_settings, rng_);
                
                // Store roadmap data
                robot_instance.roadmap.roadmap = std::make_unique<planning::Roadmap<7>>(std::move(roadmap_result));
                robot_instance.roadmap.is_built = true;
            }, robot_instance.robot_type);
        }

        /**
         * @brief Solve single robot planning problem using the correct robot type
         * @param robot_idx Index of the robot
         * @param start Start configuration
         * @param goal Goal configuration
         * @return Planning result
         */
        planning::PlanningResult<7> solve_single_robot(std::size_t robot_idx,
                                                     const vamp::FloatVector<7>& start,
                                                     const vamp::FloatVector<7>& goal)
        {
            auto& robot_instance = robots_[robot_idx];
            
            // Use std::visit to handle different robot types
            return std::visit([&](const auto& robot_type) {
                // Convert environment to SIMD format for RRTC
                auto simd_env = get_simd_environment();
                
                // Use RRTConnect for single robot planning with the correct robot type
                return planning::RRTC<std::decay_t<decltype(robot_type)>, rake, resolution>::solve(
                    start, goal, simd_env, settings_.rrtc_settings, rng_);
            }, robot_instance.robot_type);
        }
    };

}  // namespace vamp::mr_planning 