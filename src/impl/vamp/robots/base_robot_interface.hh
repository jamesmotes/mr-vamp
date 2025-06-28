#pragma once

#include <memory>
#include <string>
#include <array>
#include <vector>

#include <vamp/collision/sphere.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/fcit.hh>
#include <vamp/planning/aorrtc.hh>
#include <vamp/random/rng.hh>

namespace vamp::robots
{
    // Forward declarations
    class Configuration;
    class Path;
    class PlanningResult;
    class Roadmap;
    class RNG;
    class ProlateHyperspheroid;

    /**
     * @brief Base interface for multi-robot algorithms
     * 
     * This interface provides a common API for all robot types, enabling
     * multi-robot algorithms like Conflict-Based Search (CBS) and Prioritized Planning.
     * Each robot implementation (template-based) will be wrapped by RobotWrapper
     * to provide this interface while preserving SIMD optimizations.
     */
    class RobotInterface
    {
    public:
        virtual ~RobotInterface() = default;

        // Core robot functionality
        /**
         * @brief Compute forward kinematics for the robot
         * @param config Robot configuration
         * @return Vector of collision spheres representing the robot
         */
        virtual std::vector<collision::Sphere<float>> fk(const Configuration& config) = 0;

        /**
         * @brief Validate if a configuration is collision-free
         * @param config Robot configuration
         * @param env Environment to check against
         * @return True if configuration is valid
         */
        virtual bool validate(const Configuration& config, const collision::Environment<float>& env) = 0;

        /**
         * @brief Get the base position of the robot
         * @return Array of [x, y, z] base position coordinates
         */
        virtual std::array<float, 3> get_base_position() const = 0;

        /**
         * @brief Get the name of the robot variant
         * @return Robot name string
         */
        virtual std::string get_name() const = 0;

        // Robot properties
        /**
         * @brief Get the configuration space dimension
         * @return Number of degrees of freedom
         */
        virtual int get_dimension() const = 0;

        /**
         * @brief Get the number of collision spheres
         * @return Number of spheres in collision model
         */
        virtual int get_n_spheres() const = 0;

        /**
         * @brief Get the collision checking resolution
         * @return Resolution value
         */
        virtual float get_resolution() const = 0;

        /**
         * @brief Get the configuration space measure
         * @return Space measure value
         */
        virtual float get_space_measure() const = 0;

        // Planning methods
        /**
         * @brief Solve motion planning problem with RRTConnect
         * @param start Start configuration
         * @param goal Goal configuration
         * @param env Environment
         * @param settings RRTConnect settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult rrtc(const Configuration& start, const Configuration& goal,
                                   const collision::Environment<float>& env,
                                   const planning::RRTCSettings& settings,
                                   std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with RRTConnect (multi-goal)
         * @param start Start configuration
         * @param goals Vector of goal configurations
         * @param env Environment
         * @param settings RRTConnect settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult rrtc(const Configuration& start, const std::vector<Configuration>& goals,
                                   const collision::Environment<float>& env,
                                   const planning::RRTCSettings& settings,
                                   std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with PRM
         * @param start Start configuration
         * @param goal Goal configuration
         * @param env Environment
         * @param settings PRM settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult prm(const Configuration& start, const Configuration& goal,
                                  const collision::Environment<float>& env,
                                  const planning::RoadmapSettings<planning::PRMStarNeighborParams>& settings,
                                  std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with PRM (multi-goal)
         * @param start Start configuration
         * @param goals Vector of goal configurations
         * @param env Environment
         * @param settings PRM settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult prm(const Configuration& start, const std::vector<Configuration>& goals,
                                  const collision::Environment<float>& env,
                                  const planning::RoadmapSettings<planning::PRMStarNeighborParams>& settings,
                                  std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with FCIT*
         * @param start Start configuration
         * @param goal Goal configuration
         * @param env Environment
         * @param settings FCIT settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult fcit(const Configuration& start, const Configuration& goal,
                                   const collision::Environment<float>& env,
                                   const planning::RoadmapSettings<planning::FCITStarNeighborParams>& settings,
                                   std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with FCIT* (multi-goal)
         * @param start Start configuration
         * @param goals Vector of goal configurations
         * @param env Environment
         * @param settings FCIT settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult fcit(const Configuration& start, const std::vector<Configuration>& goals,
                                   const collision::Environment<float>& env,
                                   const planning::RoadmapSettings<planning::FCITStarNeighborParams>& settings,
                                   std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with AORRTC
         * @param start Start configuration
         * @param goal Goal configuration
         * @param env Environment
         * @param settings AORRTC settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult aorrtc(const Configuration& start, const Configuration& goal,
                                     const collision::Environment<float>& env,
                                     const planning::AORRTCSettings& settings,
                                     std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Solve motion planning problem with AORRTC (multi-goal)
         * @param start Start configuration
         * @param goals Vector of goal configurations
         * @param env Environment
         * @param settings AORRTC settings
         * @param rng Random number generator
         * @return Planning result
         */
        virtual PlanningResult aorrtc(const Configuration& start, const std::vector<Configuration>& goals,
                                     const collision::Environment<float>& env,
                                     const planning::AORRTCSettings& settings,
                                     std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Build roadmap with PRM
         * @param start Start configuration
         * @param goal Goal configuration
         * @param env Environment
         * @param settings PRM settings
         * @param rng Random number generator
         * @return Roadmap
         */
        virtual Roadmap roadmap(const Configuration& start, const Configuration& goal,
                               const collision::Environment<float>& env,
                               const planning::RoadmapSettings<planning::PRMStarNeighborParams>& settings,
                               std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Simplify a path
         * @param path Path to simplify
         * @param env Environment
         * @param settings Simplify settings
         * @param rng Random number generator
         * @return Planning result with simplified path
         */
        virtual PlanningResult simplify(const Path& path,
                                       const collision::Environment<float>& env,
                                       const planning::SimplifySettings& settings,
                                       std::shared_ptr<RNG> rng) = 0;

        /**
         * @brief Check sphere validity for a configuration
         * @param config Robot configuration
         * @param env Environment
         * @return Vector of collision information for each sphere
         */
        virtual std::vector<std::vector<std::string>> sphere_validity(const Configuration& config,
                                                                      const collision::Environment<float>& env) = 0;

        /**
         * @brief Compute end-effector forward kinematics
         * @param config Robot configuration
         * @return Pair of [position, orientation] where orientation is xyzw quaternion
         */
        virtual std::pair<std::array<float, 3>, std::array<float, 4>> eefk(const Configuration& config) = 0;

        /**
         * @brief Filter point cloud by removing points in collision with robot
         * @param pointcloud Input point cloud
         * @param config Robot configuration
         * @param env Environment
         * @param point_radius Radius of each point
         * @return Filtered point cloud
         */
        virtual std::vector<collision::Point> filter_from_pointcloud(const std::vector<collision::Point>& pointcloud,
                                                                    const Configuration& config,
                                                                    const collision::Environment<float>& env,
                                                                    float point_radius) = 0;

        /**
         * @brief Compute distance between two configurations
         * @param a First configuration
         * @param b Second configuration
         * @return L2 distance
         */
        virtual float distance(const Configuration& a, const Configuration& b) = 0;

        // RNG factory methods
        /**
         * @brief Create a Halton sequence RNG
         * @return Shared pointer to RNG
         */
        virtual std::shared_ptr<RNG> halton() = 0;

        /**
         * @brief Create a PHS sampler RNG
         * @param phs Prolate hyperspheroid
         * @param rng Base RNG
         * @return Shared pointer to RNG
         */
        virtual std::shared_ptr<RNG> phs_sampler(const ProlateHyperspheroid& phs, std::shared_ptr<RNG> rng) = 0;

#if defined(__x86_64__)
        /**
         * @brief Create an XORShift RNG (x86_64 only)
         * @return Shared pointer to RNG
         */
        virtual std::shared_ptr<RNG> xorshift() = 0;
#endif
    };

}  // namespace vamp::robots 