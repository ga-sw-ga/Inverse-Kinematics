#pragma once

#include <givr.h>

namespace rigging {

	///////////////////////////////////////////////////////////
	// This is a basis two boned arm with 3 degrees of       //
	// freedom. The base joint has two angles, one for       //
	// vertical and one for horizontal. The middle joint     //
	// has a single vertical angle. Each angle has a type of //
	// constraint, wrap (loop) and clamp (min/max), and      //
	// these constraints are applied by calling the          //
	// "applyConstraints" function. What you need to do      //
	// first is to add a third bone/joint with another       //
	// degree of freedom vertical angle (like the second     //
	// bone/joint). After this, you need to design a IK      //
	// system to automatically calculate the ANGLES (not     //
	// bone lengths) so the end effector traces the target.  //
	// Note that if the target is outside the range of the   //
	// arm, the arm should still attempt to reach (and not   //
	// make it) for the target without crashing or shake     //
	// violently. This includes to far and the special case  //
	// of then the arm cannot reach the target near the root.//
	// This usually occurs when the first length is longer   //
	// than the sum of the other arms lengths.               //
	///////////////////////////////////////////////////////////
	struct SimpleArm {
		// Simple Macros to show where these numbers are located 
		// (feel free to delete this and use number directly, just added these for some clarity)
#define NUMBER_OF_BONES 3
#define NUMBER_OF_JOINTS 3
		// Root Joint has 2 angles (vertical + horizontal)
#define NUMBER_OF_ANGLES NUMBER_OF_JOINTS + 1

		// Type name for clarity
		using joint_angles = std::array<float, NUMBER_OF_ANGLES>;
		using bone_lengths = std::array<float, NUMBER_OF_BONES>;

		// This class defines the boundrys and the protocal for enforcing the boundry 
		struct Bounds {
			enum class Type { Clamp, Wrap } type = Type::Clamp;
			float min = -std::numeric_limits<float>::quiet_NaN();
			float max = +std::numeric_limits<float>::quiet_NaN();
			// Apply the constraint to the input variable v, output depends on type and the min/max
			float apply(float v);
		};
		using joint_angle_bounds = std::array<Bounds, NUMBER_OF_ANGLES>;

		// Constructor
		SimpleArm(
			joint_angles init_angles = { 0.f, 0.f, 0.f, 0.f },
			joint_angle_bounds init_angle_bounds = {
				Bounds{Bounds::Type::Wrap, -M_PI, M_PI},
				Bounds{Bounds::Type::Clamp, 0.f, M_PI},
				Bounds{Bounds::Type::Clamp, -M_PI, M_PI},
				Bounds{Bounds::Type::Clamp, -M_PI, M_PI} //This should be your constraints for angle 4
			},
			bone_lengths init_lengths = { 5.f, 5.f, 5.f }
		);

		// Joint and end positions (should work if you change the transformation functions as expected)
		glm::vec3 jointPosition(size_t index) const;
		glm::vec3 endEffectorPosition() const;

		// Transformations (will need to modify these, though shouldnt be huge changes)
		glm::mat4 localJointM(size_t index) const;
		glm::mat4 localEndEffectorM() const;
		glm::mat4 globalJointM(size_t index) const;
		glm::mat4 globalEndEffectorM() const;

		// Ensure angles are preserved in the proper range
		void applyConstraints();

		// Generate cylinders to render the arm
		std::array<givr::geometry::Cylinder, NUMBER_OF_BONES> armGeometry(float radius);
		//------------------------------------------------------------

		// Data
		joint_angles angles;
		joint_angle_bounds angle_bounds;

		bone_lengths lengths;

		// Comes in handy throughout the program
		static const size_t number_of_joints = NUMBER_OF_JOINTS;
		static const size_t number_of_bones = NUMBER_OF_BONES;
		static const size_t number_of_angles = NUMBER_OF_ANGLES;
	};
} // namespace rigging