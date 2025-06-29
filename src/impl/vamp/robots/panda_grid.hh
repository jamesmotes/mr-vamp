#pragma once

#include <vamp/robots/panda_base.hh>

namespace vamp::robots
{
    // Grid variants (pre-compiled for maximum performance)
    // Note: Template parameters are in centimeters * 100 for precision
    // For example: 0.0f becomes 0, 1.0f becomes 100, 0.05f becomes 5
    struct Panda_0_0 : PandaBase<0, 0, 0> {
        static constexpr auto name = "panda_0_0";
    };
    struct Panda_0_1 : PandaBase<0, 100, 0> {
        static constexpr auto name = "panda_0_1";
    };
    struct Panda_0_2 : PandaBase<0, 200, 0> {
        static constexpr auto name = "panda_0_2";
    };
    struct Panda_1_0 : PandaBase<100, 0, 0> {
        static constexpr auto name = "panda_1_0";
    };
    struct Panda_1_1 : PandaBase<100, 100, 0> {
        static constexpr auto name = "panda_1_1";
    };
    struct Panda_1_2 : PandaBase<100, 200, 0> {
        static constexpr auto name = "panda_1_2";
    };
    struct Panda_2_0 : PandaBase<200, 0, 0> {
        static constexpr auto name = "panda_2_0";
    };
    struct Panda_2_1 : PandaBase<200, 100, 0> {
        static constexpr auto name = "panda_2_1";
    };
    struct Panda_2_2 : PandaBase<200, 200, 0> {
        static constexpr auto name = "panda_2_2";
    };

    // Default variant (backward compatibility)
    struct Panda : PandaBase<0, 0, 0> {
        static constexpr auto name = "panda";
    };
}  // namespace vamp::robots 