#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/mr_planning/mr_problem.hh>
#include <vamp/mr_planning/mr_settings.hh>
#include <vamp/mr_planning/mr_planner_base.hh>
#include <vamp/mr_planning/dummy_mr_planner.hh>
#include <vamp/mr_planning/mr_factory.hh>
#include <vamp/robots/panda_grid.hh>
#include <vamp/random/halton.hh>
#include <vamp/collision/environment.hh>

namespace nb = nanobind;

void vamp::binding::init_mr_planning(nb::module_ &pymodule)
{
    using namespace vamp::mr_planning;
    using Robot = vamp::robots::Panda_0_0;  // Use a specific grid robot
    static constexpr auto dimension = Robot::dimension;
    static constexpr auto rake = vamp::FloatVectorWidth;
    static constexpr auto resolution = Robot::resolution;
    
    // Create the multi-robot planning module
    auto mr_module = pymodule.def_submodule("mr_planning", "Multi-robot planning framework");
    
    // Bind MRProblem
    nb::class_<MRProblem<dimension>>(mr_module, "MRProblem")
        .def(nb::init<>(), "Create an empty multi-robot problem")
        .def("add_robot", &MRProblem<dimension>::add_robot, 
             nb::arg("start"), nb::arg("goal"), nb::arg("base_pos"), nb::arg("name") = "",
             "Add a robot to the problem")
        .def("clear", &MRProblem<dimension>::clear, "Clear all robots from the problem")
        .def("is_valid", &MRProblem<dimension>::is_valid, "Check if the problem is well-formed")
        .def("num_robots", &MRProblem<dimension>::num_robots, "Get the number of robots")
        .def_rw("start_configurations", &MRProblem<dimension>::start_configurations)
        .def_rw("goal_configurations", &MRProblem<dimension>::goal_configurations)
        .def_rw("base_positions", &MRProblem<dimension>::base_positions)
        .def_rw("environment", &MRProblem<dimension>::environment)
        .def_rw("problem_name", &MRProblem<dimension>::problem_name)
        .def_rw("robot_names", &MRProblem<dimension>::robot_names);
    
    // Bind MRPlanningResult
    nb::class_<MRPlanningResult<dimension>>(mr_module, "MRPlanningResult")
        .def(nb::init<>(), "Create an empty planning result")
        .def("is_valid", &MRPlanningResult<dimension>::is_valid, "Check if the result is valid")
        .def("num_robots", &MRPlanningResult<dimension>::num_robots, "Get the number of robot paths")
        .def("calculate_total_cost", &MRPlanningResult<dimension>::calculate_total_cost, "Calculate total cost")
        .def_rw("robot_paths", &MRPlanningResult<dimension>::robot_paths)
        .def_rw("total_cost", &MRPlanningResult<dimension>::total_cost)
        .def_rw("nanoseconds", &MRPlanningResult<dimension>::nanoseconds)
        .def_rw("iterations", &MRPlanningResult<dimension>::iterations)
        .def_rw("success", &MRPlanningResult<dimension>::success)
        .def_rw("algorithm_name", &MRPlanningResult<dimension>::algorithm_name);
    
    // Bind MRSettings
    nb::class_<MRSettings>(mr_module, "MRSettings")
        .def(nb::init<>(), "Create default multi-robot settings")
        .def_rw("enable_parallel_roadmap_construction", &MRSettings::enable_parallel_roadmap_construction)
        .def_rw("max_roadmap_size", &MRSettings::max_roadmap_size)
        .def_rw("max_roadmap_iterations", &MRSettings::max_roadmap_iterations)
        .def_rw("enable_inter_robot_collision_checking", &MRSettings::enable_inter_robot_collision_checking)
        .def_rw("inter_robot_safety_margin", &MRSettings::inter_robot_safety_margin)
        .def_rw("max_mr_iterations", &MRSettings::max_mr_iterations)
        .def_rw("algorithm", &MRSettings::algorithm)
        .def_rw("num_threads", &MRSettings::num_threads)
        .def("is_valid", &MRSettings::is_valid, "Check if settings are valid")
        .def("set_roadmap_params", &MRSettings::set_roadmap_params,
             nb::arg("max_size"), nb::arg("max_iters"), nb::arg("batch_size") = 1000,
             "Set roadmap construction parameters")
        .def("set_rrtc_params", &MRSettings::set_rrtc_params,
             nb::arg("range"), nb::arg("max_iters"), nb::arg("max_samples"),
             "Set RRTConnect parameters")
        .def("set_inter_robot_params", &MRSettings::set_inter_robot_params,
             nb::arg("enable"), nb::arg("safety_margin") = 0.1f,
             "Set inter-robot collision checking parameters")
        .def("set_performance_params", &MRSettings::set_performance_params,
             nb::arg("parallel"), nb::arg("threads") = 4,
             "Set performance parameters");
    
    // Bind MRPlannerBase (abstract base class) - disable copy constructor
    nb::class_<MRPlannerBase<Robot, rake, resolution>>(mr_module, "MRPlannerBase")
        .def("build_roadmaps", &MRPlannerBase<Robot, rake, resolution>::build_roadmaps,
             nb::arg("starts"), nb::arg("goals"), "Build roadmaps for all robots")
        .def("are_roadmaps_built", &MRPlannerBase<Robot, rake, resolution>::are_roadmaps_built,
             "Check if roadmaps are built")
        .def("solve_ignoring_inter_robot_collisions", 
             &MRPlannerBase<Robot, rake, resolution>::solve_ignoring_inter_robot_collisions,
             nb::arg("starts"), nb::arg("goals"), "Solve ignoring inter-robot collisions")
        .def("get_roadmap_build_time_ns", &MRPlannerBase<Robot, rake, resolution>::get_roadmap_build_time_ns,
             "Get roadmap build time in nanoseconds")
        .def("get_total_planning_time_ns", &MRPlannerBase<Robot, rake, resolution>::get_total_planning_time_ns,
             "Get total planning time in nanoseconds");
    
    // Bind DummyMRPlanner
    nb::class_<DummyMRPlanner<Robot, rake, resolution>, MRPlannerBase<Robot, rake, resolution>>(mr_module, "DummyMRPlanner")
        .def(nb::init<const std::vector<std::array<float, 3>>&, const vamp::collision::Environment<float>&, 
                      std::shared_ptr<vamp::rng::RNG<dimension>>, const MRSettings&>(),
             nb::arg("base_positions"), nb::arg("environment"), nb::arg("rng"), nb::arg("settings") = MRSettings(),
             "Create a dummy multi-robot planner")
        .def("solve", static_cast<MRPlanningResult<dimension>(DummyMRPlanner<Robot, rake, resolution>::*)(const std::vector<vamp::robots::Panda_0_0::Configuration>&, const std::vector<vamp::robots::Panda_0_0::Configuration>&)>(&DummyMRPlanner<Robot, rake, resolution>::solve),
             nb::arg("starts"), nb::arg("goals"), "Solve the multi-robot planning problem")
        .def("get_name", &DummyMRPlanner<Robot, rake, resolution>::get_name, "Get planner name")
        .def("get_description", &DummyMRPlanner<Robot, rake, resolution>::get_description, "Get planner description");
    
    // Bind MRPlannerFactory
    nb::class_<MRPlannerFactory>(mr_module, "MRPlannerFactory")
        .def_static("create_planner", 
                    [](const std::string& planner_type, 
                       const std::vector<std::array<float, 3>>& base_positions,
                       const vamp::collision::Environment<float>& env,
                       const MRSettings& settings,
                       std::shared_ptr<vamp::rng::RNG<dimension>> rng) -> std::unique_ptr<MRPlannerBase<Robot, rake, resolution>> {
                        return MRPlannerFactory::create_planner<Robot, rake, resolution>(
                            planner_type, base_positions, env, settings, rng);
                    },
                    nb::arg("planner_type"), nb::arg("base_positions"), nb::arg("environment"), 
                    nb::arg("settings"), nb::arg("rng"),
                    "Create a multi-robot planner of the specified type")
        .def_static("get_available_algorithms", &MRPlannerFactory::get_available_algorithms,
                    "Get list of available algorithm types")
        .def_static("is_algorithm_available", &MRPlannerFactory::is_algorithm_available,
                    nb::arg("algorithm"), "Check if an algorithm is available")
        .def_static("get_algorithm_description", &MRPlannerFactory::get_algorithm_description,
                    nb::arg("algorithm"), "Get description of an algorithm")
        .def_static("create_dummy_planner", 
                    [](const std::vector<std::array<float, 3>>& base_positions,
                       const vamp::collision::Environment<float>& env,
                       const MRSettings& settings,
                       std::shared_ptr<vamp::rng::RNG<dimension>> rng) -> std::unique_ptr<MRPlannerBase<Robot, rake, resolution>> {
                        return MRPlannerFactory::create_dummy_planner<Robot, rake, resolution>(
                            base_positions, env, settings, rng);
                    },
                    nb::arg("base_positions"), nb::arg("environment"), 
                    nb::arg("settings") = MRSettings(), nb::arg("rng") = nullptr,
                    "Create a dummy multi-robot planner");
    
    // Helper functions for easy multi-robot planning
    mr_module.def("create_mr_problem", 
                  [](const std::vector<std::vector<float>>& starts,
                     const std::vector<std::vector<float>>& goals,
                     const std::vector<std::array<float, 3>>& base_positions,
                     const vamp::collision::Environment<float>& env,
                     const std::string& problem_name = "mr_problem") -> MRProblem<dimension> {
                      MRProblem<dimension> problem;
                      problem.environment = env;
                      problem.problem_name = problem_name;
                      
                      // Convert starts and goals to FloatVector<dimension>
                      for (const auto& start : starts) {
                          if (start.size() != dimension) {
                              throw std::runtime_error("Start configuration has wrong dimension");
                          }
                          vamp::FloatVector<dimension> config;
                          for (size_t i = 0; i < dimension; ++i) {
                              config[i] = start[i];
                          }
                          problem.start_configurations.push_back(config);
                      }
                      
                      for (const auto& goal : goals) {
                          if (goal.size() != dimension) {
                              throw std::runtime_error("Goal configuration has wrong dimension");
                          }
                          vamp::FloatVector<dimension> config;
                          for (size_t i = 0; i < dimension; ++i) {
                              config[i] = goal[i];
                          }
                          problem.goal_configurations.push_back(config);
                      }
                      
                      // Convert base positions
                      for (const auto& pos : base_positions) {
                          problem.base_positions.push_back(pos);
                      }
                      
                      // Generate robot names
                      for (size_t i = 0; i < starts.size(); ++i) {
                          problem.robot_names.push_back("robot_" + std::to_string(i));
                      }
                      
                      return problem;
                  },
                  nb::arg("starts"), nb::arg("goals"), nb::arg("base_positions"), 
                  nb::arg("environment"), nb::arg("problem_name") = "mr_problem",
                  "Create a multi-robot planning problem");
    
    mr_module.def("solve_mr_problem",
                  [](const MRProblem<dimension>& problem, 
                     const std::string& planner_type = "dummy",
                     const MRSettings& settings = MRSettings()) -> MRPlanningResult<dimension> {
                      auto rng = std::make_shared<vamp::rng::Halton<dimension>>();
                      auto planner = MRPlannerFactory::create_planner<Robot, rake, resolution>(
                          planner_type, problem.base_positions, problem.environment, settings, rng);
                      
                      std::vector<vamp::robots::Panda_0_0::Configuration> starts, goals;
                      for (const auto& start : problem.start_configurations) {
                          starts.emplace_back(start);
                      }
                      for (const auto& goal : problem.goal_configurations) {
                          goals.emplace_back(goal);
                      }
                      
                      return planner->solve_ignoring_inter_robot_collisions(starts, goals);
                  },
                  nb::arg("problem"), nb::arg("planner_type") = "dummy", nb::arg("settings") = MRSettings(),
                  "Solve a multi-robot problem with the specified planner");
} 