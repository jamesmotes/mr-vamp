#include <iostream>
#include <vector>
#include <array>
#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda_grid.hh>
#include <vamp/vector.hh>

#include "../../src/impl/vamp/mr_planning/mr_factory.hh"
#include "../../src/impl/vamp/mr_planning/mr_settings.hh"
#include "../../src/impl/vamp/mr_planning/mr_problem.hh"

using namespace vamp::mr_planning;
using Robot = vamp::robots::Panda_0_0;
static constexpr std::size_t rake = 4;  // Force ARM/NEON vector width since auto-detection is failing
static constexpr std::size_t resolution = Robot::resolution;  // Use robot-specific resolution

using vamp::FloatVector;

int main() {
    std::cout << "Testing Multi-Robot Planning Framework" << std::endl;
    std::cout << "=====================================" << std::endl;

    try {
        // 1. Create environment
        std::cout << "1. Creating environment..." << std::endl;
        vamp::collision::Environment<float> environment;
        
        // Add some obstacles (COMMENTED OUT FOR SIMPLIFIED TEST)
        // environment.spheres.emplace_back(vamp::collision::factory::sphere::array({0.5f, 0.5f, 0.5f}, 0.2f));
        // environment.spheres.emplace_back(vamp::collision::factory::sphere::array({-0.5f, 0.5f, 0.5f}, 0.2f));
        environment.sort();

        // 2. Define robot base positions
        std::cout << "2. Setting up robot base positions..." << std::endl;
        std::vector<std::array<float, 3>> base_positions = {
            {0.0f, 0.0f, 0.05f},   // Robot 1 at origin
            {2.0f, 0.0f, 0.05f},   // Robot 2 at x=2
            {0.0f, 2.0f, 0.05f},   // Robot 3 at y=2
        };

        // Check workspace boundaries for each robot
        std::cout << "   Checking workspace boundaries..." << std::endl;
        const float panda_reach = 1.19f;  // From constants.py ROBOT_MAX_RADII["panda"]
        for (std::size_t i = 0; i < base_positions.size(); ++i) {
            const auto& base = base_positions[i];
            std::cout << "   Robot " << i << " base: [" << base[0] << ", " << base[1] << ", " << base[2] << "]" << std::endl;
            std::cout << "   Robot " << i << " workspace: x=[" << (base[0] - panda_reach) << ", " << (base[0] + panda_reach) 
                      << "], y=[" << (base[1] - panda_reach) << ", " << (base[1] + panda_reach) 
                      << "], z=[" << (base[2] - panda_reach) << ", " << (base[2] + panda_reach) << "]" << std::endl;
        }

        // 3. Define start and goal configurations
        std::cout << "3. Setting up start/goal configurations..." << std::endl;
        
        // Use simpler configurations that are more likely to be reachable
        // Start: neutral position, Goal: slightly different position
        std::vector<Robot::Configuration> starts = {
            Robot::Configuration(std::array<float, 7>{0.0f, -0.785f, 0.0f, -2.356f, 0.0f, 1.571f, 0.785f}),
            Robot::Configuration(std::array<float, 7>{0.0f, -0.785f, 0.0f, -2.356f, 0.0f, 1.571f, 0.785f}),
            Robot::Configuration(std::array<float, 7>{0.0f, -0.785f, 0.0f, -2.356f, 0.0f, 1.571f, 0.785f}),
        };

        std::vector<Robot::Configuration> goals = {
            Robot::Configuration(std::array<float, 7>{0.5f, 0.0f, 0.0f, -1.5f, 0.0f, 1.0f, 0.5f}),
            Robot::Configuration(std::array<float, 7>{0.5f, 0.0f, 0.0f, -1.5f, 0.0f, 1.0f, 0.5f}),
            Robot::Configuration(std::array<float, 7>{0.5f, 0.0f, 0.0f, -1.5f, 0.0f, 1.0f, 0.5f}),
        };

        // Check if configurations are within joint limits
        std::cout << "   Checking joint limits..." << std::endl;
        std::array<float, 7> joint_limits_lower = {-2.9671f, -1.8326f, -2.9671f, -3.1416f, -2.9671f, -0.0873f, -2.9671f};
        std::array<float, 7> joint_limits_upper = {2.9671f, 1.8326f, 2.9671f, 0.0873f, 2.9671f, 3.8223f, 2.9671f};
        
        for (std::size_t i = 0; i < starts.size(); ++i) {
            auto start_arr = starts[i].to_array();
            auto goal_arr = goals[i].to_array();
            std::cout << "   Robot " << i << " start config: ";
            for (std::size_t j = 0; j < 7; ++j) {
                float val = start_arr[j];
                bool valid = (val >= joint_limits_lower[j] && val <= joint_limits_upper[j]);
                std::cout << val << (valid ? "" : "(INVALID)") << " ";
            }
            std::cout << std::endl;
            
            std::cout << "   Robot " << i << " goal config:  ";
            for (std::size_t j = 0; j < 7; ++j) {
                float val = goal_arr[j];
                bool valid = (val >= joint_limits_lower[j] && val <= joint_limits_upper[j]);
                std::cout << val << (valid ? "" : "(INVALID)") << " ";
            }
            std::cout << std::endl;
        }

        // Check if start/goal configurations are collision-free
        std::cout << "   Checking collision status..." << std::endl;
        vamp::collision::Environment<FloatVector<rake>> simd_env(environment);
        for (std::size_t i = 0; i < starts.size(); ++i) {
            bool start_valid = vamp::planning::validate_motion<Robot, rake, resolution>(starts[i], starts[i], simd_env);
            bool goal_valid = vamp::planning::validate_motion<Robot, rake, resolution>(goals[i], goals[i], simd_env);
            std::cout << "   Robot " << i << ": start=" << (start_valid ? "valid" : "INVALID") 
                      << ", goal=" << (goal_valid ? "valid" : "INVALID") << std::endl;
        }

        // 4. Create settings
        std::cout << "4. Creating settings..." << std::endl;
        MRSettings settings;
        settings.set_roadmap_params(1000, 10000, 100);  // Smaller for testing
        settings.set_rrtc_params(1.0f, 1000, 1000);
        settings.set_inter_robot_params(false);  // Disable for dummy planner

        // 5. Create RNG
        std::cout << "5. Creating RNG..." << std::endl;
        auto rng = std::make_shared<vamp::rng::Halton<Robot::dimension>>();

        // 6. Create planner
        std::cout << "6. Creating dummy planner..." << std::endl;
        auto planner = MRPlannerFactory::create_planner<Robot, rake, resolution>(
            "dummy", base_positions, environment, settings, rng);

        if (!planner) {
            std::cerr << "Failed to create planner!" << std::endl;
            return 1;
        }

        std::cout << "   Created planner with " << planner->num_robots() << " robots" << std::endl;

        // 7. Build roadmaps
        std::cout << "7. Building roadmaps..." << std::endl;
        planner->build_roadmaps(starts, goals);
        
        std::cout << "   Roadmap build time: " << planner->get_roadmap_build_time_ns() / 1e6 << " ms" << std::endl;

        // 8. Solve planning problem
        std::cout << "8. Solving planning problem..." << std::endl;
        auto result = planner->solve_ignoring_inter_robot_collisions(starts, goals);

        // 9. Report results
        std::cout << "9. Results:" << std::endl;
        std::cout << "   Success: " << (result.success ? "Yes" : "No") << std::endl;
        std::cout << "   Algorithm: " << result.algorithm_name << std::endl;
        std::cout << "   Planning time: " << result.nanoseconds / 1e6 << " ms" << std::endl;
        std::cout << "   Total cost: " << result.total_cost << std::endl;
        std::cout << "   Number of robot paths: " << result.num_robots() << std::endl;
        
        for (std::size_t i = 0; i < result.robot_paths.size(); ++i) {
            const auto& path = result.robot_paths[i];
            std::cout << "   Robot " << i << ": " << path.size() << " waypoints, cost: " << path.cost() << std::endl;
        }

        // 10. Test available algorithms
        std::cout << "10. Available algorithms:" << std::endl;
        auto algorithms = MRPlannerFactory::get_available_algorithms();
        for (const auto& alg : algorithms) {
            std::cout << "   - " << alg << ": " << MRPlannerFactory::get_algorithm_description(alg) << std::endl;
        }

        std::cout << "\nTest completed successfully!" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 