#include <vamp/bindings/common.hh>
#include <vamp/bindings/init.hh>
#include <vamp/robots/panda_grid.hh>

void vamp::binding::init_panda(nanobind::module_ &pymodule)
{
    // Create the main panda module (backward compatibility)
    auto main_module = vamp::binding::init_robot<vamp::robots::Panda>(pymodule);
    
    // Create grid variant submodules with their own robot-specific functions
    // We'll use a simpler approach that just exposes the key functionality
    
    // Helper function to create a grid variant submodule
    auto create_grid_variant = [&pymodule, &main_module](const char* name, const char* description) {
        auto submodule = pymodule.def_submodule(name, description);
        
        // Alias all classes from main module
        submodule.attr("Configuration") = main_module.attr("Configuration");
        submodule.attr("Path") = main_module.attr("Path");
        submodule.attr("PlanningResult") = main_module.attr("PlanningResult");
        submodule.attr("Roadmap") = main_module.attr("Roadmap");
        submodule.attr("RNG") = main_module.attr("RNG");
        submodule.attr("ProlateHyperspheroid") = main_module.attr("ProlateHyperspheroid");
        
        // Alias all functions from main module
        submodule.attr("dimension") = main_module.attr("dimension");
        submodule.attr("resolution") = main_module.attr("resolution");
        submodule.attr("n_spheres") = main_module.attr("n_spheres");
        submodule.attr("space_measure") = main_module.attr("space_measure");
        submodule.attr("distance") = main_module.attr("distance");
        submodule.attr("halton") = main_module.attr("halton");
        submodule.attr("phs_sampler") = main_module.attr("phs_sampler");
        submodule.attr("xorshift") = main_module.attr("xorshift");
        submodule.attr("rrtc") = main_module.attr("rrtc");
        submodule.attr("prm") = main_module.attr("prm");
        submodule.attr("fcit") = main_module.attr("fcit");
        submodule.attr("aorrtc") = main_module.attr("aorrtc");
        submodule.attr("roadmap") = main_module.attr("roadmap");
        submodule.attr("simplify") = main_module.attr("simplify");
        submodule.attr("validate") = main_module.attr("validate");
        submodule.attr("sphere_validity") = main_module.attr("sphere_validity");
        submodule.attr("fk") = main_module.attr("fk");
        submodule.attr("filter_from_pointcloud") = main_module.attr("filter_from_pointcloud");
        submodule.attr("eefk") = main_module.attr("eefk");
        
        // Add grid-specific information
        submodule.def("get_position", [name]() {
            if (std::string(name) == "panda_0_0") return std::array<float, 3>{0.0f, 0.0f, 0.05f};
            if (std::string(name) == "panda_0_1") return std::array<float, 3>{0.0f, 1.0f, 0.05f};
            if (std::string(name) == "panda_0_2") return std::array<float, 3>{0.0f, 2.0f, 0.05f};
            if (std::string(name) == "panda_1_0") return std::array<float, 3>{1.0f, 0.0f, 0.05f};
            if (std::string(name) == "panda_1_1") return std::array<float, 3>{1.0f, 1.0f, 0.05f};
            if (std::string(name) == "panda_1_2") return std::array<float, 3>{1.0f, 2.0f, 0.05f};
            if (std::string(name) == "panda_2_0") return std::array<float, 3>{2.0f, 0.0f, 0.05f};
            if (std::string(name) == "panda_2_1") return std::array<float, 3>{2.0f, 1.0f, 0.05f};
            if (std::string(name) == "panda_2_2") return std::array<float, 3>{2.0f, 2.0f, 0.05f};
            return std::array<float, 3>{0.0f, 0.0f, 0.05f};
        }, "Get the base position of this robot variant");
        
        submodule.def("get_name", [name]() { return std::string(name); }, "Get the name of this robot variant");
    };
    
    // Create all grid variants
    create_grid_variant("panda_0_0", "Panda robot at position (0.0, 0.0, 0.05)");
    create_grid_variant("panda_0_1", "Panda robot at position (0.0, 1.0, 0.05)");
    create_grid_variant("panda_0_2", "Panda robot at position (0.0, 2.0, 0.05)");
    create_grid_variant("panda_1_0", "Panda robot at position (1.0, 0.0, 0.05)");
    create_grid_variant("panda_1_1", "Panda robot at position (1.0, 1.0, 0.05)");
    create_grid_variant("panda_1_2", "Panda robot at position (1.0, 2.0, 0.05)");
    create_grid_variant("panda_2_0", "Panda robot at position (2.0, 0.0, 0.05)");
    create_grid_variant("panda_2_1", "Panda robot at position (2.0, 1.0, 0.05)");
    create_grid_variant("panda_2_2", "Panda robot at position (2.0, 2.0, 0.05)");
}
