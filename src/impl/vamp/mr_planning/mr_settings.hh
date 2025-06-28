#pragma once

#include <string>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/rrtc_settings.hh>

namespace vamp::mr_planning
{
    /**
     * @brief Settings for multi-robot planning algorithms
     * 
     * This structure contains all the configuration parameters needed for
     * multi-robot planning, including roadmap construction, algorithm selection,
     * and performance tuning.
     */
    struct MRSettings
    {
        // Roadmap construction settings
        planning::RoadmapSettings<planning::PRMStarNeighborParams> roadmap_settings;
        std::size_t max_roadmap_size = 10000;
        std::size_t max_roadmap_iterations = 100000;
        
        // Multi-robot specific settings
        bool enable_inter_robot_collision_checking = true;
        float inter_robot_safety_margin = 0.1f;
        std::size_t max_mr_iterations = 1000;
        
        // Algorithm selection
        std::string algorithm = "dummy";  // "dummy", "cbs", "prioritized", etc.
        
        // Performance settings
        bool enable_parallel_roadmap_construction = true;
        std::size_t num_threads = 4;
        
        // RRTConnect settings (for single-robot planning within multi-robot)
        planning::RRTCSettings rrtc_settings;
        
        /**
         * @brief Default constructor with reasonable defaults
         */
        MRSettings() : roadmap_settings(planning::PRMStarNeighborParams(7, 1000.0)) {
            // Set up default roadmap settings
            roadmap_settings.max_iterations = max_roadmap_iterations;
            roadmap_settings.max_samples = max_roadmap_size;
            roadmap_settings.batch_size = 1000;
            roadmap_settings.optimize = false;
            
            // Set up default RRTConnect settings
            rrtc_settings.range = 1.0f;
            rrtc_settings.max_iterations = 10000;
            rrtc_settings.max_samples = 10000;
        }
        
        /**
         * @brief Validate that settings are reasonable
         * @return True if settings are valid
         */
        bool is_valid() const {
            return max_roadmap_size > 0 &&
                   max_roadmap_iterations > 0 &&
                   max_mr_iterations > 0 &&
                   inter_robot_safety_margin >= 0.0f &&
                   num_threads > 0 &&
                   !algorithm.empty();
        }
        
        /**
         * @brief Set roadmap construction parameters
         * @param max_size Maximum number of nodes in roadmap
         * @param max_iters Maximum iterations for roadmap construction
         * @param batch_size Batch size for sampling
         */
        void set_roadmap_params(std::size_t max_size, std::size_t max_iters, std::size_t batch_size = 1000) {
            max_roadmap_size = max_size;
            max_roadmap_iterations = max_iters;
            roadmap_settings.max_iterations = max_iters;
            roadmap_settings.max_samples = max_size;
            roadmap_settings.batch_size = batch_size;
        }
        
        /**
         * @brief Set RRTConnect parameters for single-robot planning
         * @param range Range for RRTConnect
         * @param max_iters Maximum iterations
         * @param max_samples Maximum samples
         */
        void set_rrtc_params(float range, std::size_t max_iters, std::size_t max_samples) {
            rrtc_settings.range = range;
            rrtc_settings.max_iterations = max_iters;
            rrtc_settings.max_samples = max_samples;
        }
        
        /**
         * @brief Set inter-robot collision checking parameters
         * @param enable Whether to enable inter-robot collision checking
         * @param safety_margin Safety margin for collision detection
         */
        void set_inter_robot_params(bool enable, float safety_margin = 0.1f) {
            enable_inter_robot_collision_checking = enable;
            inter_robot_safety_margin = safety_margin;
        }
        
        /**
         * @brief Set performance parameters
         * @param parallel Whether to enable parallel roadmap construction
         * @param threads Number of threads to use
         */
        void set_performance_params(bool parallel, std::size_t threads = 4) {
            enable_parallel_roadmap_construction = parallel;
            num_threads = threads;
        }
    };

}  // namespace vamp::mr_planning 