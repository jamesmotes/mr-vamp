#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/shared_ptr.h>

#include <vamp/mr_planning/mr_problem.hh>
#include <vamp/mr_planning/mr_settings.hh>
#include <vamp/mr_planning/mr_planner_base.hh>
#include <vamp/mr_planning/dummy_mr_planner.hh>
#include <vamp/mr_planning/mr_factory.hh>
#include <vamp/collision/environment.hh>
#include <vamp/random/rng.hh>
#include <vamp/random/halton.hh>
#include <vamp/vector.hh>

namespace nb = nanobind;

using namespace vamp::mr_planning;

// Type aliases for Python bindings
using MRPlanner = MRPlannerBase<4, 4>;
using DummyPlanner = DummyMRPlanner<4, 4>;
using Environment = vamp::collision::Environment<float>;
using RNG = vamp::rng::RNG<7>;
using HaltonRNG = vamp::rng::Halton<7>;
using FloatVector7 = vamp::FloatVector<7>;

namespace vamp::binding
{

void init_mr_planning(nb::module_& pymodule)
{
    // Create the multi-robot planning module
    auto mr_module = pymodule.def_submodule("mr_planning", "Multi-robot planning framework");

    // Bind FloatVector<7> for configurations
    nb::class_<FloatVector7>(mr_module, "FloatVector7")
        .def(nb::init<>())
        .def(nb::init<const std::vector<float>&>())
        .def("__getitem__", [](const FloatVector7& v, std::size_t i) {
            if (i >= 7) throw nb::index_error("Index out of range");
            return v[{i, 0}];
        })
        .def("__setitem__", [](FloatVector7& v, std::size_t i, float value) {
            if (i >= 7) throw nb::index_error("Index out of range");
            // Create new FloatVector with modified data
            auto array = v.to_array();
            array[i] = value;
            v = FloatVector7(array.data());
        })
        .def("to_list", [](const FloatVector7& v) {
            std::vector<float> result(7);
            for (std::size_t i = 0; i < 7; ++i) {
                result[i] = v[{i, 0}];
            }
            return result;
        })
        .def("from_list", [](FloatVector7& v, const std::vector<float>& values) {
            if (values.size() != 7) throw std::runtime_error("Expected 7 values");
            // Create new FloatVector from the values
            v = FloatVector7(values.data());
        });
    // Explicitly assign FloatVector7 to the submodule's namespace
    mr_module.attr("FloatVector7") = nb::type<FloatVector7>();

    // Bind MRProblem
    nb::class_<MRProblem<7>>(mr_module, "MRProblem")
        .def(nb::init<>(), "Create an empty multi-robot problem")
        .def("add_robot", &MRProblem<7>::add_robot, 
             nb::arg("start"), nb::arg("goal"), nb::arg("base_pos"), nb::arg("name") = "",
             "Add a robot to the problem")
        .def("clear", &MRProblem<7>::clear, "Clear all robots from the problem")
        .def("is_valid", &MRProblem<7>::is_valid, "Check if the problem is well-formed")
        .def("num_robots", &MRProblem<7>::num_robots, "Get the number of robots")
        .def_rw("start_configurations", &MRProblem<7>::start_configurations)
        .def_rw("goal_configurations", &MRProblem<7>::goal_configurations)
        .def_rw("base_positions", &MRProblem<7>::base_positions)
        .def_rw("environment", &MRProblem<7>::environment)
        .def_rw("problem_name", &MRProblem<7>::problem_name)
        .def_rw("robot_names", &MRProblem<7>::robot_names);
    
    // Bind MRPlanningResult
    nb::class_<MRPlanningResult<7>>(mr_module, "MRPlanningResult")
        .def(nb::init<>(), "Create an empty planning result")
        .def("is_valid", &MRPlanningResult<7>::is_valid, "Check if the result is valid")
        .def("num_robots", &MRPlanningResult<7>::num_robots, "Get the number of robot paths")
        .def("calculate_total_cost", &MRPlanningResult<7>::calculate_total_cost, "Calculate total cost")
        .def_rw("robot_paths", &MRPlanningResult<7>::robot_paths)
        .def_rw("total_cost", &MRPlanningResult<7>::total_cost)
        .def_rw("nanoseconds", &MRPlanningResult<7>::nanoseconds)
        .def_rw("iterations", &MRPlanningResult<7>::iterations)
        .def_rw("success", &MRPlanningResult<7>::success)
        .def_rw("algorithm_name", &MRPlanningResult<7>::algorithm_name);
    
    // Bind MRSettings
    nb::class_<MRSettings>(mr_module, "MRSettings")
        .def(nb::init<>(), "Create default multi-robot settings")
        .def_rw("enable_parallel_roadmap_construction", &MRSettings::enable_parallel_roadmap_construction)
        .def_rw("enable_inter_robot_collision_checking", &MRSettings::enable_inter_robot_collision_checking)
        .def_rw("inter_robot_safety_margin", &MRSettings::inter_robot_safety_margin)
        .def_rw("algorithm", &MRSettings::algorithm)
        .def_rw("roadmap_settings", &MRSettings::roadmap_settings)
        .def_rw("rrtc_settings", &MRSettings::rrtc_settings);
    
    // Bind MRPlannerBase (abstract base class)
    nb::class_<MRPlanner>(mr_module, "MRPlannerBase")
        .def("build_roadmaps", &MRPlanner::build_roadmaps)
        .def("are_roadmaps_built", &MRPlanner::are_roadmaps_built)
        .def("solve_ignoring_inter_robot_collisions", 
             &MRPlanner::solve_ignoring_inter_robot_collisions)
        .def("get_roadmap_build_time_ns", &MRPlanner::get_roadmap_build_time_ns)
        .def("get_total_planning_time_ns", &MRPlanner::get_total_planning_time_ns)
        .def("get_settings", &MRPlanner::get_settings, nb::rv_policy::reference)
        .def("get_environment", &MRPlanner::get_environment, nb::rv_policy::reference);
    
    // Bind DummyMRPlanner
    nb::class_<DummyPlanner, MRPlanner>(mr_module, "DummyMRPlanner")
        .def(nb::init<const std::vector<std::array<float, 3>>&, 
                      const vamp::collision::Environment<float>&, 
                      std::shared_ptr<vamp::rng::RNG<7>>, 
                      const MRSettings&>(),
             nb::arg("base_positions"), nb::arg("env"), nb::arg("rng"), nb::arg("settings") = MRSettings(),
             "Create a dummy multi-robot planner")
        .def("solve", static_cast<MRPlanningResult<7> (DummyPlanner::*)(const std::vector<FloatVector7>&, const std::vector<FloatVector7>&)>(&DummyPlanner::solve),
             nb::arg("starts"), nb::arg("goals"), "Solve the multi-robot planning problem");
    
    // Bind MRPlannerFactory
    nb::class_<MRPlannerFactory>(mr_module, "MRPlannerFactory")
        .def_static("create_planner", &MRPlannerFactory::create_planner,
                    nb::arg("algorithm"), nb::arg("base_positions"), nb::arg("environment"), 
                    nb::arg("rng"), nb::arg("settings") = MRSettings(),
                    "Create a multi-robot planner of the specified type")
        .def_static("get_available_algorithms", &MRPlannerFactory::get_available_algorithms,
                    "Get list of available planning algorithms")
        .def_static("is_algorithm_available", &MRPlannerFactory::is_algorithm_available,
                    nb::arg("algorithm"), "Check if an algorithm is available");
    
    // Helper functions for easy multi-robot planning
    mr_module.def("create_mr_problem", 
                  [](const std::vector<std::array<float, 3>>& base_positions,
                     const std::vector<FloatVector7>& starts,
                     const std::vector<FloatVector7>& goals,
                     const vamp::collision::Environment<float>& env,
                     const MRSettings& settings) -> MRProblem<7> {
                      MRProblem<7> problem;
                      problem.environment = env;
                      problem.problem_name = "mr_problem";
                      
                      problem.base_positions = base_positions;
                      
                      problem.start_configurations = starts;
                      problem.goal_configurations = goals;
                      
                      // Generate robot names
                      for (std::size_t i = 0; i < base_positions.size(); ++i) {
                          problem.robot_names.push_back("robot_" + std::to_string(i));
                      }
                      
                      return problem;
                  },
                  nb::arg("base_positions"), nb::arg("starts"), nb::arg("goals"), 
                  nb::arg("environment"), nb::arg("settings"),
                  "Create a multi-robot planning problem");

    mr_module.def("solve_mr_problem",
                  [](const MRProblem<7>& problem, 
                     const std::string& algorithm = "dummy",
                     const MRSettings& settings = MRSettings()) -> MRPlanningResult<7> {
                      // Create RNG using concrete implementation
                      auto rng = std::make_shared<HaltonRNG>();
                      
                      // Create planner
                      auto planner = MRPlannerFactory::create_planner(
                          algorithm, problem.base_positions, problem.environment, rng, settings);
                      
                      // Solve problem
                      return planner->solve_ignoring_inter_robot_collisions(
                          problem.start_configurations, problem.goal_configurations);
                  },
                  nb::arg("problem"), nb::arg("algorithm") = "dummy", nb::arg("settings") = MRSettings(),
                  "Solve a multi-robot planning problem");

    // Helper function to create dummy planner
    mr_module.def("create_dummy_planner", [](const std::vector<std::array<float, 3>>& base_positions,
                                            const vamp::collision::Environment<float>& env,
                                            const MRSettings& settings) {
        auto rng = std::make_shared<HaltonRNG>();
        return std::make_unique<DummyPlanner>(base_positions, env, rng, settings);
    });

    // Set __all__ for the submodule
    mr_module.attr("__all__") = nb::make_tuple(
        "FloatVector7",
        "MRProblem",
        "MRPlanningResult",
        "MRSettings",
        "MRPlannerBase",
        "DummyMRPlanner",
        "MRPlannerFactory",
        "create_mr_problem",
        "solve_mr_problem",
        "create_dummy_planner"
    );
}

}  // namespace vamp::binding 