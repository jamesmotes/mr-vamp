#pragma once

namespace vamp::robots
{
    // Note: Using integer template parameters instead of float due to compiler limitations.
    // AppleClang (and some other compilers) do not yet support non-type template parameters
    // of type 'float' even with C++20. This is a known limitation.
    // 
    // Template parameters represent base positions in centimeters * 100 for precision.
    // For example: BaseX100 = 150 means base_x = 1.5 meters
    template<int BaseX100, int BaseY100, int BaseZ100>
    struct BaseRobot {
        static constexpr float base_x = static_cast<float>(BaseX100) / 100.0f;
        static constexpr float base_y = static_cast<float>(BaseY100) / 100.0f;
        static constexpr float base_z = static_cast<float>(BaseZ100) / 100.0f;
    };
}  // namespace vamp::robots 