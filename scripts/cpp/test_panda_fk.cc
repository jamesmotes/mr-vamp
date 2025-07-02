#include <iostream>
#include <vamp/robots/panda_base.hh>
#include <vamp/collision/environment.hh>
#include <vamp/vector.hh>

using namespace vamp;

int main() {
    std::cout << "Testing Panda FK with base offsets..." << std::endl;
    
    // Test configuration (all zeros for simplicity)
    vamp::robots::panda::Configuration q_single;
    q_single[0] = 0.0f;
    q_single[1] = 0.0f;
    q_single[2] = 0.0f;
    q_single[3] = 0.0f;
    q_single[4] = 0.0f;
    q_single[5] = 0.0f;
    q_single[6] = 0.0f;
    
    // Create ConfigurationBlock for SIMD processing - fill with zeros for all lanes
    vamp::robots::panda::ConfigurationBlock<vamp::FloatVectorWidth> q;
    // Initialize all SIMD lanes with the same zero configuration
    for (size_t i = 0; i < vamp::FloatVectorWidth; ++i) {
        q[0][i] = 0.0f;
        q[1][i] = 0.0f;
        q[2][i] = 0.0f;
        q[3][i] = 0.0f;
        q[4][i] = 0.0f;
        q[5][i] = 0.0f;
        q[6][i] = 0.0f;
    }
    
    // Test both robots
    vamp::robots::PandaBase<0, 0, 0> panda_origin;
    vamp::robots::PandaBase<200, 200, 0> panda_offset;  // 2m, 2m, 0m offset
    
    // Get sphere positions for both robots - use correct SIMD width
    vamp::robots::panda::Spheres<vamp::FloatVectorWidth> spheres_origin, spheres_offset;
    
    panda_origin.sphere_fk(q, spheres_origin);
    panda_offset.sphere_fk(q, spheres_offset);
    
    std::cout << "\n=== Sphere Positions ===" << std::endl;
    std::cout << "Robot at (0,0,0) - First 5 spheres:" << std::endl;
    for (int i = 0; i < 5; i++) {
        std::cout << "  Sphere " << i << ": (" 
                  << spheres_origin.x[i] << ", " 
                  << spheres_origin.y[i] << ", " 
                  << spheres_origin.z[i] << ") r=" 
                  << spheres_origin.r[i] << std::endl;
    }
    
    std::cout << "\nRobot at (2,2,0) - First 5 spheres:" << std::endl;
    for (int i = 0; i < 5; i++) {
        std::cout << "  Sphere " << i << ": (" 
                  << spheres_offset.x[i] << ", " 
                  << spheres_offset.y[i] << ", " 
                  << spheres_offset.z[i] << ") r=" 
                  << spheres_offset.r[i] << std::endl;
    }
    
    // Print actual FK sphere centers for both robots (first sphere)
    std::cout << "\n=== FK Sphere Centers (First Sphere) ===" << std::endl;
    std::cout << "Robot at (0,0,0): ("
              << spheres_origin.x[0] << ", "
              << spheres_origin.y[0] << ", "
              << spheres_origin.z[0] << ") r=" << spheres_origin.r[0] << std::endl;
    std::cout << "Robot at (2,2,0): ("
              << spheres_offset.x[0] << ", "
              << spheres_offset.y[0] << ", "
              << spheres_offset.z[0] << ") r=" << spheres_offset.r[0] << std::endl;
    
    // Test collision detection
    std::cout << "\n=== Collision Detection Tests ===" << std::endl;
    
    // Create environment with some test spheres - use float instead of SIMD vectors
    vamp::collision::Environment<float> env;
    
    // Add some test spheres - use proper constructor
    env.spheres.emplace_back(5.0f, 5.0f, 0.5f, 0.3f);  // Far sphere - should not collide
    env.spheres.emplace_back(0.05f, 0.05f, 0.05f, 0.1f);  // Very close sphere - should collide with origin robot
    env.spheres.emplace_back(2.05f, 2.05f, 0.05f, 0.1f);  // Very close sphere - should collide with offset robot
    
    // Sort environment for proper collision detection
    env.sort();
    
    std::cout << "Environment spheres:" << std::endl;
    for (size_t i = 0; i < env.spheres.size(); i++) {
        std::cout << "  Sphere " << i << ": (" 
                  << env.spheres[i].x << ", " 
                  << env.spheres[i].y << ", " 
                  << env.spheres[i].z << ") r=" 
                  << env.spheres[i].r 
                  << " min_dist=" << env.spheres[i].min_distance << std::endl;
    }
    
    // Print environment sphere centers
    std::cout << "\n=== Environment Sphere Centers ===" << std::endl;
    for (size_t i = 0; i < env.spheres.size(); ++i) {
        std::cout << "Env Sphere " << i << ": ("
                  << env.spheres[i].x << ", "
                  << env.spheres[i].y << ", "
                  << env.spheres[i].z << ") r=" << env.spheres[i].r << std::endl;
    }
    
    // Convert to SIMD environment
    vamp::collision::Environment<FloatVector<vamp::FloatVectorWidth>> env_simd(env);
    
    // Debug: Print the converted SIMD environment spheres
    std::cout << "\n=== Converted SIMD Environment Spheres ===" << std::endl;
    for (size_t i = 0; i < env_simd.spheres.size(); ++i) {
        const auto& sphere = env_simd.spheres[i];
        std::cout << "SIMD Sphere " << i << ": (";
        std::cout << sphere.x << ", " << sphere.y << ", " << sphere.z << ") r=" << sphere.r;
        std::cout << " min_dist=" << sphere.min_distance << std::endl;
    }
    
    // Test collision detection
    std::cout << "\n=== Collision Detection Tests ===" << std::endl;
    bool collision_origin = panda_origin.fkcc(env_simd, q);
    bool collision_offset = panda_offset.fkcc(env_simd, q);
    
    std::cout << "Collision results:" << std::endl;
    std::cout << "Robot at (0,0,0): " << (!collision_origin ? "COLLISION" : "NO COLLISION") << std::endl;
    std::cout << "Robot at (2,2,0): " << (!collision_offset ? "COLLISION" : "NO COLLISION") << std::endl;
    
    // Test manual collision detection to verify the issue
    std::cout << "\n=== Manual Collision Test ===" << std::endl;
    // Test if a sphere at (0.1, 0.1, 0.5) with radius 0.1 collides with robot at (0,0,0)
    // The robot's first sphere should be at (0, 0, 0.05) with radius 0.08
    float robot_sphere_x = 0.0f + panda_origin.base_x;
    float robot_sphere_y = 0.0f + panda_origin.base_y;
    float robot_sphere_z = 0.05f + panda_origin.base_z;
    float robot_sphere_r = 0.08f;
    
    float env_sphere_x = 0.05f;
    float env_sphere_y = 0.05f;
    float env_sphere_z = 0.05f;
    float env_sphere_r = 0.1f;
    
    // Calculate distance between sphere centers
    float dx = robot_sphere_x - env_sphere_x;
    float dy = robot_sphere_y - env_sphere_y;
    float dz = robot_sphere_z - env_sphere_z;
    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    float min_distance = robot_sphere_r + env_sphere_r;
    
    std::cout << "Manual collision check:" << std::endl;
    std::cout << "  Robot sphere: (" << robot_sphere_x << ", " << robot_sphere_y << ", " << robot_sphere_z << ") r=" << robot_sphere_r << std::endl;
    std::cout << "  Env sphere: (" << env_sphere_x << ", " << env_sphere_y << ", " << env_sphere_z << ") r=" << env_sphere_r << std::endl;
    std::cout << "  Distance: " << distance << ", Min distance for collision: " << min_distance << std::endl;
    std::cout << "  Should collide: " << (distance < min_distance ? "YES" : "NO") << std::endl;
    
    return 0;
} 