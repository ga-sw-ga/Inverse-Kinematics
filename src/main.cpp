#include <givio.h>
#include <givr.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>
#include <cmath>
#include <limits>

#include "picking_controls.h"
#include "turntable_controls.h"

#include "imgui_panel.hpp"
#include "simple_arm.hpp"
#include "skinned_model.hpp"

#include <glm/gtx/string_cast.hpp>
#include <thread>
#include <chrono>

using namespace giv;
using namespace giv::io;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;


glm::mat4x3 multiply_mat4_by_mat4x3(const glm::mat4& mat4x4, const glm::mat4x3& mat4x3) {
    glm::mat4x3 result;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = 0.f;
            for (int k = 0; k < 4; ++k) {
                result[i][j] += mat4x4[i][k] * mat4x3[k][j];
            }
        }
    }

    return result;
}

glm::vec4 multiply_mat4x3_by_vec3(const glm::mat4x3& mat4x3, const glm::vec3& vec3) {
    glm::vec4 result;

    for (int i = 0; i < 4; ++i) {
        result[i] = 0.f;
        for (int j = 0; j < 3; ++j) {
            result[i] += mat4x3[i][j] * vec3[j];
        }
    }

    return result;
}


glm::vec3 project(glm::vec3 e_t, glm::vec3 e, float delta) {
    glm::vec3 d = e_t - e;
    float l = glm::length(d);
    if (l > delta) {
        return e + (delta / l) * d;
    }
    else {
        return e_t;
    }
}


glm::mat3x4 jacobian(rigging::SimpleArm& arm, const glm::vec4& angle, const float epsilon) {
    glm::mat3x4 jac = glm::mat3x4();
    glm::vec4 e = glm::vec4(0.f, 0.f, 0.f, 0.f);
    for (int j = 0; j < 4; j++) {
        e = glm::vec4(0.f, 0.f, 0.f, 0.f);
        e[j] = epsilon;
        glm::vec4 theta_plus_epsilon = angle + e;
        glm::vec4 theta_minus_epsilon = angle - e;
        glm::vec3 p_plus_theta = arm.calculateEndEffectorPosition(theta_plus_epsilon);
        glm::vec3 p_minus_theta = arm.calculateEndEffectorPosition(theta_minus_epsilon);
        glm::vec3 d_f_over_d_theta = (p_plus_theta - p_minus_theta) / (2.f * epsilon);
        for (int i = 0; i < 3; i++) {
            jac[i][j] = d_f_over_d_theta[i];
        }
    }
    return jac;
}

// Damped (λ) least squares method
glm::vec4 solve_delta_theta_dls(const glm::mat3x4& j_theta, const glm::vec3& delta_e, const float& lambda) {
    glm::vec4 delta_theta;

    glm::mat4x3 j_theta_t = glm::transpose(j_theta);
    glm::mat4 j_theta_t_times_j_theta = j_theta_t * j_theta;
    glm::mat4 lambda_squared_times_identity = (lambda * lambda) * glm::identity<glm::mat4>();
    glm::mat4 term_inside_parentheses = j_theta_t_times_j_theta + lambda_squared_times_identity;
    glm::mat4 inverse_term = glm::inverse(term_inside_parentheses);
    glm::mat4x3 inverse_term_times_j_theta_t = multiply_mat4_by_mat4x3(inverse_term, j_theta_t);
    delta_theta = multiply_mat4x3_by_vec3(inverse_term_times_j_theta_t, delta_e);

    return delta_theta;
}


glm::vec4 solve_delta_theta_jt(const glm::mat3x4& j_theta, const glm::vec3& delta_e) {
    glm::vec4 delta_theta;

    glm::mat4x3 j_theta_t = glm::transpose(j_theta);
    glm::mat3 j_theta_times_j_theta_t = j_theta * j_theta_t;
    glm::vec3 j_theta_times_j_theta_t_times_delta_e = j_theta_times_j_theta_t * delta_e;
    float alpha = glm::dot(delta_e, j_theta_times_j_theta_t_times_delta_e) / glm::dot(j_theta_times_j_theta_t_times_delta_e, j_theta_times_j_theta_t_times_delta_e);
    delta_theta = alpha * multiply_mat4x3_by_vec3(j_theta_t, delta_e);

    return delta_theta;
}


std::array<float, 4> vec4_to_array(const glm::vec4 vector) {
    return { vector.x, vector.y, vector.z, vector.w };
}


// program entry point
int main(void) {
    // important variables initialized
    glm::vec3 e, e_t, e_delta_t, delta_e;
    glm::vec4 theta = glm::vec4(0.f, 0.f, 0.f, 0.f);
    glm::vec4 delta_theta = glm::vec4(0.f, 0.f, 0.f, 0.f);
    glm::mat3x4 j_theta = glm::mat3x4(1.f);
    float angle_epsilon = 0.01f, delta = 0.01f, lambda = 0.1f, position_epsilon = 0.05f, stopwatch = 0.f;

    // skinning
    std::array<glm::mat4, 3> jointRestTransforms{
            glm::mat4(1.0f), // Identity matrix for joint 0
            glm::mat4(1.0f), // Identity matrix for joint 1
            glm::mat4(1.0f)  // Identity matrix for joint 2
    };

    std::array<glm::mat4, 3> jointPosedTransforms{
            glm::mat4(1.0f), // Identity matrix for joint 0
            glm::mat4(1.0f), // Identity matrix for joint 1
            glm::mat4(1.0f)  // Identity matrix for joint 2
    };


    // benchmark
//    int benchmark_index = 0;
//    bool change_benchmark = true;
//    glm::vec3 benchmark_positions[5] = {glm::vec3(5.f, 5.f, 5.f),
//                                        glm::vec3(-5.f, 5.f, 0.f),
//                                        glm::vec3(5.f, 5.f, -5.f),
//                                        glm::vec3(5.f, 6.f, 5.f),
//                                        glm::vec3(3.f, 6.f, -4.f)};

	// initialize OpenGL and window
	GLFWContext glContext;
	glContext.glMajorVesion(3)
		.glMinorVesion(3)
		.glForwardComaptability(true)
		.glCoreProfile()
		.glAntiAliasingSamples(4)
		.matchPrimaryMonitorVideoMode();
	std::cout << glfwVersionString() << '\n';

	// setup window (OpenGL context)
	ImGuiWindow window = glContext.makeImGuiWindow(Properties()
		.size(dimensions{ 1000, 1000 })
		.title("Inverse Kinematics")
		.glslVersionString("#version 330 core"));
	// set our imgui update function
	panel::update_lambda_function = imgui_panel::draw;

	ViewContext view = View(TurnTable(), Perspective());
	TurnTableControls controls(window, view.camera);
	view.camera.latitude() = M_PI / 4.f; //So you can see downward initially

	//Used for IK solution
	glm::vec3 target(0.f);

	// Curser Routine
	bool follow_mouse = false;
	CursorPosition curser_pos = { 0, 0 };
	window.cursorCommand() = [&](auto pixel) { curser_pos = pixel; };

	// Bind keys
	auto toggle_panel_routine = [&](auto event) {
		if (event.action == GLFW_PRESS)
			imgui_panel::showPanel = !imgui_panel::showPanel;
	};
	auto close_window_routine     = [&](auto) { window.shouldClose(); };
	auto increment_rot_px_routine = [&](auto) { view.camera.rotateAroundX(0.1f); };
	auto increment_rot_nx_routine = [&](auto) { view.camera.rotateAroundX(-0.1f); };
	auto increment_rot_py_routine = [&](auto) { view.camera.rotateAroundY(0.1f); };
	auto increment_rot_ny_routine = [&](auto) { view.camera.rotateAroundY(-0.1f); };
	auto follow_mouse_routine = [&](auto event) { 		
		if (event.action == GLFW_PRESS || event.action == GLFW_REPEAT)
			follow_mouse = true;
		else if (event.action == GLFW_RELEASE)
			follow_mouse = false;
	};
	window.keyboardCommands()
		| Key(GLFW_KEY_ESCAPE,	close_window_routine)
		| Key(GLFW_KEY_P,		toggle_panel_routine)
		| Key(GLFW_KEY_LEFT,	increment_rot_nx_routine)
		| Key(GLFW_KEY_RIGHT,	increment_rot_px_routine)
		| Key(GLFW_KEY_UP,		increment_rot_ny_routine)
		| Key(GLFW_KEY_DOWN,	increment_rot_py_routine)
		| Key(GLFW_KEY_SPACE,	follow_mouse_routine);

	// Plane render
	const glm::vec3 corner_A(-1.f, 0.f, -1.f);
	const glm::vec3 corner_B(1.f,  0.f, -1.f);
	const glm::vec3 corner_C(1.f,  0.f, 1.f);
	const glm::vec3 corner_D(-1.f, 0.f, 1.f);
	Mesh plate_geometry = Mesh(Filename("./models/plate.obj"));
	Phong plate_style
		= Phong(Colour(1., 0., 0.1529), AmbientFactor(0.1), LightPosition(2., 15., 2.));
	RenderContext plate_render = createRenderable(plate_geometry, plate_style);

	// Sphere render
	Sphere sphere_geometry = Sphere(Radius(1.));
	Phong sphere_style 
		= Phong(Colour(1., 1., 0.1529), AmbientFactor(0.1), LightPosition(2., 15., 2.));
	InstancedRenderContext spheres_renders 
		= createInstancedRenderable(sphere_geometry, sphere_style);

	// Bone render
	Cylinder bone_geometry = Cylinder(Point1(0, 0, 0), Point2(0, 1, 0)); // kind of silly, need dummy cylinder
	Phong bone_style
		= Phong(Colour(1., 1., 0.1529), AmbientFactor(0.1), LightPosition(2., 15., 2.));
	RenderContext bone_render = createRenderable(bone_geometry, bone_style);

	rigging::SimpleArm arm; //Default Geometry
	skinning::SkinnedModel model = skinning::SkinnedModel::loadFromFile("./models/bone_mesh_weights.txt").value();

	// Model render
	TriangleSoup model_geometry = model.makeMesh();
	Phong  model_style = Phong(Colour(1., 1., 0.1529), AmbientFactor(0.1), LightPosition(2., 15., 2.));
	RenderContext model_render = createRenderable(model_geometry, model_style);

	// main loop
	mainloop(std::move(window), [&](float dt /**** Time since last frame ****/) {
        stopwatch += dt;

        // Reset to pose
        if (imgui_panel::reset_pose) {
            arm.resetToRest();
            imgui_panel::joint_angles = std::array<float, 4>{0.f, 0.f, 0.f, 0.f};
            imgui_panel::reset_pose = false;
        }

        lambda = imgui_panel::lambda;
        position_epsilon = imgui_panel::stopping_distance;
        angle_epsilon = imgui_panel::finite_angle_step;
        delta = imgui_panel::project_distance;

		// ----- Target ----- //
		//Simple function to animate the target in a interesting fashion
		if (imgui_panel::animate_target) {
			static float t = 0.f;
			static float factor = 1.f;
			t += factor * dt;
			if (t < 0.f || t > 10.f) {
				t = std::clamp(t, 0.f, 10.f);
				factor *= -1.f;
			}
			//Override target position
			target = glm::vec3{
				0.f + 9.f * std::cos(.5f * M_PI * t),
				5.f + 1.f * std::sin(5.f * M_PI * t),
				0.f - 3.f * std::sin(.5f * M_PI * t)
			};
		}

        // For benchmark
//        if (change_benchmark && benchmark_index < 5) {
//            target = benchmark_positions[benchmark_index];
//            change_benchmark = false;
//            benchmark_index++;
//        }
        //Move the target in the view plane by tracking the curser
		else if (follow_mouse) {
			glm::mat4 MVP = view.projection.projectionMatrix() * view.camera.viewMatrix();
			glm::mat4 invMVP = inverse(MVP);
			glm::vec3 normalized_coordinates = world3DToNDC(target, MVP);
			target = pixelToWorld3D(
				curser_pos.x, curser_pos.y, 
				window.width(), window.height(), 
				invMVP, normalized_coordinates.z
			);
		}
        e_t = target;
        e = arm.endEffectorPosition();
        theta = glm::vec4(arm.angles[0], arm.angles[1], arm.angles[2], arm.angles[3]);

		// ----- Kinematics ----- //
		if (imgui_panel::isIK) {
			//TO-DO: your IK process(s) should be here
            int iteration = 0;
            while (glm::length(e_t - e) > position_epsilon && iteration < 100) {
                e = arm.endEffectorPosition();
                e_t = target;
                e_delta_t = project(e_t, e, delta);
                delta_e = e_delta_t - e;
                j_theta = jacobian(arm, theta, angle_epsilon);
                if (imgui_panel::isBonus) {
                    delta_theta = solve_delta_theta_dls(j_theta, delta_e, lambda);
                }
                else {
                    delta_theta = solve_delta_theta_jt(j_theta, delta_e);
                }
                theta += delta_theta;
                arm.angles = vec4_to_array(theta);
                iteration++;
            }
            // for benchmark
//            if (glm::length(e_t - e) <= position_epsilon && !change_benchmark) {
//                change_benchmark = true;
//                std::cout << "Benchmark " << benchmark_index << " Completed!\nTime: " << stopwatch << "\n";
//            }

			arm.applyConstraints();//Called somewhere in here
		}
		else {
			//Simply apply the inputs
			arm.angles = imgui_panel::joint_angles;
			arm.applyConstraints();
			imgui_panel::joint_angles = arm.angles;
		}


		// ----- Skinning ----- //
		if (imgui_panel::isLBS) {
			//The bones are defined by the model
			arm.lengths[0] = model.bones[0].length;
			arm.lengths[1] = model.bones[1].length;
			arm.lengths[2] = model.bones[2].length;
			imgui_panel::bone_lengths = arm.lengths;

			//TO-DO: Calculate tranformations to use below in Update
            for (size_t i = 0; i < arm.number_of_joints; ++i) {
                // Joint rest transform
                jointRestTransforms[i] = arm.getJointRestTransform(i);

                // Joint posed transform
                jointPosedTransforms[i] = arm.getJointPosedTransform(i);
            }
		}
		else {
			arm.lengths = imgui_panel::bone_lengths;
		}

		// ----- Rendering ----- //
		auto color = imgui_panel::clear_color;
		glClearColor(color.x, color.y, color.z, color.z);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		view.projection.updateAspectRatio(window.width(), window.height());
		
		if (imgui_panel::isLBS) {
			model.updateMesh(
				model_geometry,
				{jointRestTransforms[0], jointRestTransforms[1], jointRestTransforms[2]},
				{jointPosedTransforms[0], jointPosedTransforms[1], jointPosedTransforms[2]}
			);
			updateRenderable(model_geometry, model_style, model_render);
			draw(model_render, view, glm::mat4(1.f));
		}
		else {
			//Bones
			for (auto const& bone : arm.armGeometry(0.1f)) {
				updateRenderable(bone, bone_style, bone_render);
				draw(bone_render, view);
			}
			//Joints
			for (int i = 0; i < arm.number_of_joints; i++)
				addInstance(spheres_renders, glm::translate(arm.jointPosition(i)) * glm::scale(vec3f(0.3f)));
			//End Effector
			addInstance(spheres_renders, glm::translate(arm.endEffectorPosition()) * glm::scale(vec3f(0.3f)));
		}
		//Target
		addInstance(spheres_renders, glm::translate(target) * glm::scale(vec3f(0.2f)));
		draw(spheres_renders, view);//Also the joints and end effector

		draw(plate_render, view, glm::scale(glm::vec3(arm.lengths[0])));
	});
	return EXIT_SUCCESS;
}