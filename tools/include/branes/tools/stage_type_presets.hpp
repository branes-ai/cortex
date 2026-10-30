// SPDX-License-Identifier: MIT
//
// branes/tools/stage_type_presets.hpp — named stage-type policies for the
// MSCKF backend, shared by the EuRoC tools' --stage-types option (issue #450).
//
// The state is stored in double in every preset, so a tool's front end, I/O
// and metrics are unchanged; a preset names the type each stage computes in
// (sdk/msckf/stage_types.hpp). Native types only: an emulated type end to end
// is ~10^4x slower.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_STAGE_TYPE_PRESETS_HPP
#define BRANES_TOOLS_STAGE_TYPE_PRESETS_HPP

#include <branes/sdk/msckf/stage_types.hpp>

#include <string_view>
#include <type_traits>

namespace branes::tools {

namespace stage_types {
using Double = sdk::msckf::UniformStageTypes<double>;
using Float = sdk::msckf::UniformStageTypes<float>;
using LongDouble = sdk::msckf::UniformStageTypes<long double>;
struct SensorModelFloat : Double {
    using sensor_model = float;
};
struct PropagationFloat : Double {
    using propagation = float;
};
struct AugmentationFloat : Double {
    using augmentation = float;
};
struct UpdateFloat : Double {
    using update = float;
};
struct MarginalizationFloat : Double {
    using marginalization = float;
};
struct CalibrationFloat : Double {
    using calibration = float;
};
struct FloatUpdateDouble : Float {
    using update = double;
};
struct EkfUpdateFloat : Double {
    using ekf_update = float;
};
struct UpdateFloatEkfDouble : Double {
    using update = float;
    using ekf_update = double;
};
struct FloatEkfDouble : Float {
    using ekf_update = double;
};
}  // namespace stage_types

struct StageTypePreset {
    std::string_view name;
    std::string_view summary;
};
// The order matches with_stage_types below.
inline constexpr StageTypePreset kStageTypePresets[] = {
    {"double", "every stage in double (the default)"},
    {"float", "every stage in float"},
    {"long-double", "every stage in long double"},
    {"sensor-model-float", "S0 in float"},
    {"propagation-float", "S2 in float"},
    {"augmentation-float", "S3 in float"},
    {"update-float", "S5 -> S6e in float"},
    {"marginalization-float", "S9 in float"},
    {"calibration-float", "S10 in float (runs only with online calibration, off by default)"},
    {"float-update-double", "every stage in float except S5 -> S6e"},
    {"ekf-update-float", "S6e (the EKF update alone) in float"},
    {"update-float-ekf-double", "S5 -> S6d in float, S6e in double"},
    {"float-ekf-double", "every stage in float except S6e"},
};

inline bool is_stage_type_preset(std::string_view name) {
    for (const auto& p : kStageTypePresets)
        if (p.name == name)
            return true;
    return false;
}

/// f(std::type_identity<Policy>{}) for the preset `name`. Callers validate the
/// name with is_stage_type_preset; any other name runs the double preset.
template <class F>
decltype(auto) with_stage_types(std::string_view name, F&& f) {
    namespace st = stage_types;
    if (name == "float")
        return f(std::type_identity<st::Float>{});
    if (name == "long-double")
        return f(std::type_identity<st::LongDouble>{});
    if (name == "sensor-model-float")
        return f(std::type_identity<st::SensorModelFloat>{});
    if (name == "propagation-float")
        return f(std::type_identity<st::PropagationFloat>{});
    if (name == "augmentation-float")
        return f(std::type_identity<st::AugmentationFloat>{});
    if (name == "update-float")
        return f(std::type_identity<st::UpdateFloat>{});
    if (name == "marginalization-float")
        return f(std::type_identity<st::MarginalizationFloat>{});
    if (name == "calibration-float")
        return f(std::type_identity<st::CalibrationFloat>{});
    if (name == "float-update-double")
        return f(std::type_identity<st::FloatUpdateDouble>{});
    if (name == "ekf-update-float")
        return f(std::type_identity<st::EkfUpdateFloat>{});
    if (name == "update-float-ekf-double")
        return f(std::type_identity<st::UpdateFloatEkfDouble>{});
    if (name == "float-ekf-double")
        return f(std::type_identity<st::FloatEkfDouble>{});
    return f(std::type_identity<st::Double>{});
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_STAGE_TYPE_PRESETS_HPP
