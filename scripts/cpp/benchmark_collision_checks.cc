#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>

#include <vamp/robots/panda_grid.hh>
#include <vamp/collision/environment.hh>
#include <vamp/vector.hh>
#include <vamp/robots/panda_base.hh>

using namespace vamp;
using namespace vamp::robots;

int main(int argc, char* argv[]) {
    // Parse command line arguments
    int n_trials = 100;
    int n_configurations = 1000;
    
    if (argc >= 2) n_trials = std::stoi(argv[1]);
    if (argc >= 3) n_configurations = std::stoi(argv[2]);
    
    std::cout << "=== VAMP Collision Detection Benchmark ===" << std::endl;
    std::cout << "Trials: " << n_trials << std::endl;
    std::cout << "Configurations per trial: " << n_configurations << std::endl;
    std::cout << "Total collision checks: " << (n_trials * n_configurations) << std::endl;
    std::cout << std::endl;
    
    // Create environment with sphere cage from Python script
    collision::Environment<float> env;
    
    // Sphere cage problem from sphere_cage_example.py
    std::vector<std::array<float, 3>> sphere_centers = {
        {0.55f, 0.0f, 0.25f},
        {0.35f, 0.35f, 0.25f},
        {0.0f, 0.55f, 0.25f},
        {-0.55f, 0.0f, 0.25f},
        {-0.35f, -0.35f, 0.25f},
        {0.0f, -0.55f, 0.25f},
        {0.35f, -0.35f, 0.25f},
        {0.35f, 0.35f, 0.8f},
        {0.0f, 0.55f, 0.8f},
        {-0.35f, 0.35f, 0.8f},
        {-0.55f, 0.0f, 0.8f},
        {-0.35f, -0.35f, 0.8f},
        {0.0f, -0.55f, 0.8f},
        {0.35f, -0.35f, 0.8f}
    };
    
    float sphere_radius = 0.2f;
    
    // Add spheres to environment
    for (const auto& center : sphere_centers) {
        env.spheres.emplace_back(center[0], center[1], center[2], sphere_radius);
    }
    
    // Sort environment for proper collision detection
    env.sort();
    
    std::cout << "Environment created with " << env.spheres.size() << " spheres" << std::endl;
    std::cout << "Sphere radius: " << sphere_radius << std::endl;
    std::cout << std::endl;
    
    // Convert to SIMD environment
    collision::Environment<FloatVector<FloatVectorWidth>> env_simd(env);
    
    // Create Panda robots at different base positions
    PandaBase<0, 0, 0> panda_origin;
    PandaBase<200, 200, 0> panda_offset;  // At (2, 2, 0)
    
    // Set up random number generator with fixed seed
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<float> joint_dist(-M_PI, M_PI);
    
    // Configuration block for storing joint values
    vamp::robots::panda::ConfigurationBlock<FloatVectorWidth> q;
    
    // Benchmark results storage
    std::vector<double> collision_times;
    std::vector<double> validation_times;
    std::vector<int> collision_results;
    
    collision_times.reserve(n_trials * n_configurations);
    validation_times.reserve(n_trials * n_configurations);
    collision_results.reserve(n_trials * n_configurations);
    
    std::cout << "Starting benchmark..." << std::endl;
    
    auto total_start = std::chrono::high_resolution_clock::now();
    
    for (int trial = 0; trial < n_trials; ++trial) {
        for (int config = 0; config < n_configurations; ++config) {
            // Generate random configuration
            for (int i = 0; i < 7; ++i) {
                q[i] = joint_dist(rng);
            }
            
            // Test collision detection for origin robot
            auto collision_start = std::chrono::high_resolution_clock::now();
            bool collision_result = panda_origin.fkcc(env_simd, q);
            auto collision_end = std::chrono::high_resolution_clock::now();
            
                // Test validation (which includes collision detection)
    auto validation_start = std::chrono::high_resolution_clock::now();
    bool validation_result = panda_origin.fkcc(env_simd, q);  // Use fkcc instead of validate
    auto validation_end = std::chrono::high_resolution_clock::now();
            
            // Calculate durations in microseconds
            auto collision_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                collision_end - collision_start).count() / 1000.0;
            auto validation_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                validation_end - validation_start).count() / 1000.0;
            
            collision_times.push_back(collision_duration);
            validation_times.push_back(validation_duration);
            collision_results.push_back(collision_result ? 0 : 1);  // 0 = no collision, 1 = collision
        }
        
        // Progress indicator
        if ((trial + 1) % 10 == 0) {
            std::cout << "Completed " << (trial + 1) << "/" << n_trials << " trials" << std::endl;
        }
    }
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        total_end - total_start).count();
    
    // Calculate statistics
    auto calculate_stats = [](const std::vector<double>& times) {
        if (times.empty()) return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0);
        
        auto sorted_times = times;
        std::sort(sorted_times.begin(), sorted_times.end());
        
        double min_val = sorted_times.front();
        double max_val = sorted_times.back();
        double median = sorted_times[sorted_times.size() / 2];
        double mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        
        double variance = 0.0;
        for (double time : times) {
            variance += (time - mean) * (time - mean);
        }
        variance /= times.size();
        double std_dev = std::sqrt(variance);
        
        return std::make_tuple(min_val, max_val, mean, median, std_dev);
    };
    
    auto collision_stats = calculate_stats(collision_times);
    auto validation_stats = calculate_stats(validation_times);
    
    // Count collision results
    int total_collisions = std::accumulate(collision_results.begin(), collision_results.end(), 0);
    double collision_rate = static_cast<double>(total_collisions) / collision_results.size() * 100.0;
    
    // Print results
    std::cout << std::endl;
    std::cout << "=== Benchmark Results ===" << std::endl;
    std::cout << "Total time: " << total_duration << " ms" << std::endl;
    std::cout << "Collision rate: " << std::fixed << std::setprecision(2) 
              << collision_rate << "% (" << total_collisions << "/" << collision_results.size() << ")" << std::endl;
    std::cout << std::endl;
    
    std::cout << "=== Collision Detection Performance (μs) ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Min:     " << std::get<0>(collision_stats) << std::endl;
    std::cout << "Max:     " << std::get<1>(collision_stats) << std::endl;
    std::cout << "Mean:    " << std::get<2>(collision_stats) << std::endl;
    std::cout << "Median:  " << std::get<3>(collision_stats) << std::endl;
    std::cout << "Std Dev: " << std::get<4>(collision_stats) << std::endl;
    std::cout << std::endl;
    
    std::cout << "=== Validation Performance (μs) ===" << std::endl;
    std::cout << "Min:     " << std::get<0>(validation_stats) << std::endl;
    std::cout << "Max:     " << std::get<1>(validation_stats) << std::endl;
    std::cout << "Mean:    " << std::get<2>(validation_stats) << std::endl;
    std::cout << "Median:  " << std::get<3>(validation_stats) << std::endl;
    std::cout << "Std Dev: " << std::get<4>(validation_stats) << std::endl;
    std::cout << std::endl;
    
    // Calculate throughput
    double avg_collision_time = std::get<2>(collision_stats);
    double avg_validation_time = std::get<2>(validation_stats);
    
    std::cout << "=== Throughput ===" << std::endl;
    std::cout << "Collision checks per second: " << (1000000.0 / avg_collision_time) << std::endl;
    std::cout << "Validations per second: " << (1000000.0 / avg_validation_time) << std::endl;
    
    return 0;
} 