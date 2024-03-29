#include "imgui_panel.hpp"

namespace imgui_panel {
    // default values
    bool showPanel = true;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // rig
    std::array<float, 3> bone_lengths = { 5.f, 5.f, 5.f };
    std::array<float, 4> joint_angles = { 0.f, 0.f, 0.f, 0.f };

    // Bonus content
    bool isBonus = false;

    //Kinematics
    bool isIK = false;

    // skinning
    bool isLBS = false;

    // animation
    bool animate_target = false;

    // reset pose
    bool reset_pose = false;

    // New parameters for additional features
    float finite_angle_step = 0.01f;
    float project_distance = 0.01f;
    float stopping_distance = 0.05f;
    float lambda = 0.1f;

    std::function<void(void)> draw = [](void) {
        if (showPanel && ImGui::Begin("Panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
			ImGui::Spacing();

			ImGui::Text(
				"Input Controls: \n"
				"    - Arrows control camera \n"
				"    - Holding space tracks the target to the mouse\n"
				"    - P key toggles panel\n"
				"    - Esc key quits program"
			);

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::ColorEdit3("Clear color", (float*)&clear_color);

            ImGui::Spacing();
            ImGui::Separator();

            // Enable Bonus Content checkbox
            ImGui::Checkbox("Enable Bonus Content", &isBonus);

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::Checkbox("Animate Target Override", &animate_target);
			ImGui::Checkbox("Use Inverse Kinematics", &isIK);

            // Lambda
            if (isBonus && isIK) {
                ImGui::SliderFloat("Lambda", &lambda, 0.01f, 1.0f);
            }

            // Finite Angle Step (h_theta)
            if (isIK) {
                ImGui::SliderFloat("Finite Angle Step (h_theta)", &finite_angle_step, 0.001f, 0.1f);
            }

            // Project Distance (delta)
            if (isIK) {
                ImGui::SliderFloat("Project Distance (delta)", &project_distance, 0.001f, 0.1f);
            }

            // Stopping Distance (eps)
            if (isIK) {
                ImGui::SliderFloat("Stopping Distance (eps)", &stopping_distance, 0.01f, 1.f);
            }

			ImGui::Checkbox("Use Skinning Model", &isLBS);

			ImGui::Spacing();
			ImGui::Separator();

            if (ImGui::Button("Reset to Rest")) {
                reset_pose = true;
            }

			// Model has a specific bone length to use and thus the input should be disabled
			if (!isLBS) {
				ImGui::SliderFloat("Bone Length 0", &bone_lengths[0], 0.f, 12.f);
				ImGui::SliderFloat("Bone Length 1", &bone_lengths[1], 0.f, 12.f);
                ImGui::SliderFloat("Bone Length 2", &bone_lengths[2], 0.f, 12.f);
			}

			// No forward kinematics under IK and thus the input should be disabled
			if (!isIK) {
				float joint_angle_0_deg = glm::degrees(joint_angles[0]);
				ImGui::DragFloat("Joint 0 Horizontal Angle", &joint_angle_0_deg, 1.f, -10000.f, 10000.f, "%.0f");
				joint_angles[0] = glm::radians(joint_angle_0_deg);
				//These sliders operate/renders in DEGREES but the input variable (joint_angles[i]) is RAD (convertes internally)
				// https://github.com/ocornut/imgui/issues/1435
				ImGui::SliderAngle("Joint 0 Vertical Angle", &joint_angles[1], 0.f, 180.f);
				ImGui::SliderAngle("Joint 1 Vertical Angle", &joint_angles[2], -180.f, 180.f);
                ImGui::SliderAngle("Joint 2 Vertical Angle", &joint_angles[3], -180.f, 180.f);
			}

			ImGui::Spacing();
			ImGui::Separator();
			float frame_rate = ImGui::GetIO().Framerate;
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
				1000.0f / frame_rate, frame_rate);

			ImGui::End();
		}
	};
} // namespace panel