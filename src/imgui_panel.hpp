#pragma once

#include <givio.h>
#include <givr.h>
#include <imgui/imgui.h>

namespace imgui_panel {
	extern bool showPanel;
	extern ImVec4 clear_color;

	// rig
	extern std::array<float, 3> bone_lengths; //Will need to change this after adding the bone
	extern std::array<float, 4> joint_angles; //Will need to change this after adding the joint

	// kinematics
	extern bool isIK;

	// skinning
	extern bool isLBS;

	// animation
	extern bool animate_target;

	// lambda function
	extern std::function<void(void)> draw;
} // namespace panel