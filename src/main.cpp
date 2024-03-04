#include <givio.h>
#include <givr.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>

#include "picking_controls.h"
#include "turntable_controls.h"

#include "imgui_panel.hpp"
#include "simple_arm.hpp"
#include "skinned_model.hpp"

using namespace giv;
using namespace giv::io;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

// program entry point
int main(void) {
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

		// ----- Kinematics ----- //
		if (imgui_panel::isIK) {
			//TO-DO: your IK process(s) should be here
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
				{/*TODO: Define the joint rest transformation*/},
				{/*TODO: Define the joint posed transformation*/}
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