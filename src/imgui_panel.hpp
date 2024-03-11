#pragma once

#include <givio.h>
#include <givr.h>
#include <imgui/imgui.h>

namespace imgui_panel {
    extern bool showPanel;
    extern ImVec4 clear_color;

    // rig
    extern std::array<float, 3> bone_lengths;
    extern std::array<float, 4> joint_angles;

    // bonus
    extern bool isBonus;

    // kinematics
    extern bool isIK;

    // skinning
    extern bool isLBS;

    // animation
    extern bool animate_target;

    // reset pose
    extern bool reset_pose;

    // New parameters for additional features
    extern float finite_angle_step;
    extern float project_distance;
    extern float stopping_distance;
    extern float lambda;

    // lambda function
    extern std::function<void(void)> draw;
} // namespace panel
