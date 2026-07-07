#pragma once

#include <array>
#include <cstddef>
#include <functional>

// Feature D: live arm-pose sliders in the MuJoCo UI. The slider backing values
// live here (mjUI sliders bind to mjtNum = double). simulate.cc adds the section
// + detects slider edits; main.cc installs the DDS publish hook. On any edit we
// build the 14-dim arm pose and publish it to rt/arm_pose_cmd (External mode).
namespace arm_gui {

// Slider backing (radians; [4] = slew seconds). Symmetric L/R.
//   [0] shoulder_pitch (negative = forward)   [1] shoulder_roll (abduction, mirrored)
//   [2] shoulder_yaw                           [3] elbow (higher = straighter)
//   [4] transition_s (slew time for the controller)
// Initial values = the default rest arm pose.
inline std::array<double, 5>& sliders() {
  static std::array<double, 5> s = {0.4, 0.0, 0.0, 0.3, 2.0};
  return s;
}

// main.cc installs the actual DDS publish here: (14-dim pose, transition_s).
inline std::function<void(const std::array<float, 14>&, float)>& publish_fn() {
  static std::function<void(const std::array<float, 14>&, float)> f;
  return f;
}

// True if p points into the slider backing array (used by UiEvent to recognise
// an arm-slider edit without depending on the section index).
inline bool owns(const void* p) {
  const double* base = sliders().data();
  return p >= base && p < base + sliders().size();
}

// Build the 14-dim arm pose from the sliders and publish it.
// 14-dim order: shPitch L R, shRoll L R, shYaw L R, elbPitch L R,
//               elbRoll L R, wrPitch L R, wrYaw L R.
inline void publish_from_sliders() {
  const auto& s = sliders();
  std::array<float, 14> pose{};
  pose[0] = pose[1] = static_cast<float>(s[0]);   // shoulder_pitch L,R
  pose[2] = static_cast<float>(s[1]);              // shoulder_roll L (+abduct)
  pose[3] = static_cast<float>(-s[1]);             // shoulder_roll R (mirrored)
  pose[4] = pose[5] = static_cast<float>(s[2]);    // shoulder_yaw L,R
  pose[6] = pose[7] = static_cast<float>(s[3]);    // elbow_pitch L,R
  // elbow_roll, wrist_pitch, wrist_yaw left at 0
  if (publish_fn()) publish_fn()(pose, static_cast<float>(s[4]));
}

}  // namespace arm_gui
