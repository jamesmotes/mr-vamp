#pragma once

#include <memory>
#include <string>
#include <array>
#include <vector>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/fcit.hh>
#include <vamp/planning/aorrtc.hh>
#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    // Use actual types instead of forward declarations
    using Configuration = vamp::FloatVector<7>;  // Fixed dimension for Panda robots
    using Path = vamp::planning::Path<7>;
    using PlanningResult = vamp::planning::PlanningResult<7>;
    using Roadmap = vamp::planning::Roadmap<7>;
    using RNG = vamp::rng::RNG<7>;
    using ProlateHyperspheroid = vamp::planning::ProlateHyperspheroid<7>;

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
         * @brief Build roadmap for the robot
         * @param start Start configuration
         * @param goal Goal configuration
         * @param env Environment
         * @param settings Roadmap settings
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
         * @brief Get sphere validity information
         * @param config Configuration
         * @param env Environment
         * @return Vector of collision information for each sphere
         */
        virtual std::vector<std::vector<std::string>> sphere_validity(const Configuration& config,
                                                                     const collision::Environment<float>& env) = 0;

        /**
         * @brief Compute end-effector forward kinematics
         * @param config Configuration
         * @return Pair of position and orientation
         */
        virtual std::pair<std::array<float, 3>, std::array<float, 4>> eefk(const Configuration& config) = 0;

        /**
         * @brief Filter robot from pointcloud
         * @param pointcloud Input pointcloud
         * @param config Configuration
         * @param env Environment
         * @param point_radius Point radius
         * @return Filtered pointcloud
         */
        virtual std::vector<collision::Point> filter_from_pointcloud(const std::vector<collision::Point>& pointcloud,
                                                                    const Configuration& config,
                                                                    const collision::Environment<float>& env,
                                                                    float point_radius) = 0;

        /**
         * @brief Compute distance between configurations
         * @param a First configuration
         * @param b Second configuration
         * @return Distance
         */
        virtual float distance(const Configuration& a, const Configuration& b) = 0;

        // RNG factory methods
        /**
         * @brief Create Halton sequence RNG
         * @return Shared pointer to RNG
         */
        virtual std::shared_ptr<RNG> halton() = 0;

        /**
         * @brief Create PHS sampler
         * @param phs Prolate hyperspheroid
         * @param rng Base RNG
         * @return Shared pointer to PHS RNG
         */
        virtual std::shared_ptr<RNG> phs_sampler(const ProlateHyperspheroid& phs, std::shared_ptr<RNG> rng) = 0;

#if defined(__x86_64__)
        /**
         * @brief Create XORShift RNG
         * @return Shared pointer to RNG
         */
        virtual std::shared_ptr<RNG> xorshift() = 0;
#endif
    };

}  // namespace vamp::robots 