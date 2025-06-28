#pragma once

#include <vamp/robots/base_robot.hh>
#include <vamp/robots/panda/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    // Note: Using integer template parameters instead of float due to compiler limitations.
    // AppleClang (and some other compilers) do not yet support non-type template parameters
    // of type 'float' even with C++20. This is a known limitation.
    // 
    // Template parameters represent base positions in centimeters * 100 for precision.
    // For example: BaseX100 = 150 means base_x = 1.5 meters
    template<int BaseX100, int BaseY100, int BaseZ100>
    struct PandaBase : BaseRobot<BaseX100, BaseY100, BaseZ100>
    {
        static constexpr auto name = "panda";
        static constexpr auto dimension = 7;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = panda::n_spheres;
        static constexpr auto space_measure = panda::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = panda::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = panda::Spheres<rake>;

        static constexpr auto scale_configuration = panda::scale_configuration;
        static constexpr auto descale_configuration = panda::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = panda::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = panda::descale_configuration_block<rake>;

        // Use templated FK functions with base position parameters
        // This allows compile-time optimization of base position additions
        template <std::size_t rake>
        static constexpr auto sphere_fk = panda::sphere_fk<rake, base_x, base_y, base_z>;

        template <std::size_t rake>
        static constexpr auto fkcc = panda::interleaved_sphere_fk<rake, base_x, base_y, base_z>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = panda::interleaved_sphere_fk_attachment<rake, base_x, base_y, base_z>;

        static constexpr auto eefk = panda::eefk;
    };
}  // namespace vamp::robots 