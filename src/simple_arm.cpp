#include "simple_arm.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>
#include <stdexcept>
#include <algorithm>


namespace rigging {

    float SimpleArm::Bounds::apply(float v) {
        if (std::isnan(min) || std::isinf(min) || std::isnan(max) || std::isinf(max))
            throw std::runtime_error("Bounds must be defined and finite");

        switch (type) {
            case Bounds::Type::Clamp: {
                v = std::clamp(v, min, max);
            } break;
            case Bounds::Type::Wrap: {
                //Relative to the lower bound
                float shifted_bound = max - min;
                float shifted_v		= v   - min;
                //Modulate the result in range
                shifted_v = std::fmod(shifted_v, shifted_bound);
                if (shifted_v < 0.f) //Correct for negative result
                    shifted_v = std::fmod(shifted_bound + shifted_v, shifted_bound);
                //Return to relative to origin (0)
                v = shifted_v + min;
            } break;
        }
        return v;
    }

	SimpleArm::SimpleArm(
		joint_angles init_angles,
		joint_angle_bounds init_angle_bounds,
		bone_lengths init_lengths
	)
		: angles(init_angles)
		, angle_bounds(init_angle_bounds)
		, lengths(init_lengths)
	{}

	glm::vec3 SimpleArm::jointPosition(size_t index) const {
		return glm::vec3(globalJointM(index) * glm::vec4{ 0.f, 0.f, 0.f, 1.f });
	}
	glm::vec3 SimpleArm::endEffectorPosition() const {
		return glm::vec3(globalEndEffectorM() * glm::vec4{ 0.f, 0.f, 0.f, 1.f });
	}

    glm::mat4 SimpleArm::localJointM(size_t index) const {
        switch (index) {
            case 0: { // Root joint at the base of the first bone
                // Horizontal rotation ("around" the origin along the plane using the y axis)
                glm::mat4 R_bone_0_y = glm::rotate(angles[0], glm::vec3{ 0.f, 1.f, 0.f });
                // Vertical rotation (rotate up/down using the z axis)
                glm::mat4 R_bone_0_z = glm::rotate(angles[1], glm::vec3{ 0.f, 0.f, 1.f });
                return R_bone_0_y * R_bone_0_z; // The y rotation happens last (left most)
            } break;
            case 1: { // Joint at the base of the second bone/ tip of the first bone
                glm::mat4 T_bone_0 = glm::translate(glm::vec3{ lengths[0], 0.f, 0.f }); // Along X axis is rest
                // Vertical rotation (rotate up/down using the z axis)
                glm::mat4 R_bone_1_z = glm::rotate(angles[2], glm::vec3{ 0.f, 0.f, 1.f });
                return T_bone_0 * R_bone_1_z; // The translation happens last after the local rotation
            } break;
            case 2: { // Joint at the tip of the arm
                glm::mat4 T_bone_1 = glm::translate(glm::vec3{ lengths[1], 0.f, 0.f }); // Along X axis is rest
                // Vertical rotation (rotate up/down using the z axis)
                glm::mat4 R_bone_2_z = glm::rotate(angles[3], glm::vec3{ 0.f, 0.f, 1.f });
                return T_bone_1 * R_bone_2_z; // The translation happens last after the local rotation
            } break;
        }

        throw std::runtime_error("Not a valid joint");
        return {};
    }

	glm::mat4 SimpleArm::localEndEffectorM() const {
		return glm::translate(glm::vec3{ lengths[2], 0.f, 0.f }); // Add the last bone length. Make sure to change this
	}

	glm::mat4 SimpleArm::globalJointM(size_t index) const {
		switch (index) {
		case 0: { //Root joint at the base of the first bone
			return localJointM(0);
		} break;
		case 1: { //Joint at the base of the second bone/ tip of the first bone
			return localJointM(0) * localJointM(1);
		} break;
		case 2: { //Joint at the base of the third bone/ tip of the second bone
            return localJointM(0) * localJointM(1) * localJointM(2);
		} break;
		}

		throw std::runtime_error("Not a valid joint");
		return {};
	}

	glm::mat4 SimpleArm::globalEndEffectorM() const {
		return localJointM(0) * localJointM(1) * localJointM(2) * localEndEffectorM();
	}

	void SimpleArm::applyConstraints() {
		for (size_t i = 0; i < angle_bounds.size(); i++)
			angles[i] = angle_bounds[i].apply(angles[i]);
		//We assume the lengths are positive or 0 as they are not transitional units
	}

    std::array<givr::geometry::Cylinder, NUMBER_OF_BONES> SimpleArm::armGeometry(float radius) {
        std::array<givr::geometry::Cylinder, NUMBER_OF_BONES> geom = {
                givr::geometry::Cylinder(
                        givr::geometry::Point1(jointPosition(0)),
                        givr::geometry::Point2(jointPosition(1)),
                        givr::geometry::Radius(radius)
                ),
                givr::geometry::Cylinder(
                        givr::geometry::Point1(jointPosition(1)),
                        givr::geometry::Point2(jointPosition(2)),
                        givr::geometry::Radius(radius)
                ),
                givr::geometry::Cylinder(
                        givr::geometry::Point1(jointPosition(2)),
                        givr::geometry::Point2(endEffectorPosition()), // this will change
                        givr::geometry::Radius(radius)
                )
        };
        return geom;
    }

    glm::vec3 SimpleArm::calculateEndEffectorPosition(const glm::vec4& theta) const {
        // Create a copy of the current arm
        SimpleArm tempArm = *this;

        // Set the provided angles for calculation
        tempArm.angles = {theta[0], theta[1], theta[2], theta[3]}; // Set the angles to the provided theta
        tempArm.applyConstraints(); // Ensure the angles are within bounds

        // Calculate end effector position for the temporary arm
        return tempArm.endEffectorPosition();
    }

    glm::mat4 SimpleArm::getJointRestTransform(size_t index) const {
        switch (index) {
            case 0: { // Root joint at the base of the first bone
                return glm::mat4(1.0f); // Identity matrix for the root joint
            } break;
            case 1: { // Joint at the base of the second bone/ tip of the first bone
                return glm::translate(glm::vec3(lengths[0], 0.f, 0.f)); // Translation along X axis
            } break;
            case 2: { // Joint at the tip of the arm
                return glm::translate(glm::vec3(lengths[0] + lengths[1], 0.f, 0.f)); // Translation along X axis
            } break;
        }

        throw std::runtime_error("Not a valid joint");
        return glm::mat4(1.0f); // Return identity matrix for safety
    }

    glm::mat4 SimpleArm::getJointPosedTransform(size_t index) const {
        // You can return the global transformation matrix for each joint as it is affected by all previous joints
        return globalJointM(index);
    }

    // Function to reset the arm to its rest position
    void SimpleArm::resetToRest() {
        // Reset angles to their initial values
        angles = std::array<float, 4>{0.f, 0.f, 0.f, 0.f};

        // Apply constraints to ensure angles are within bounds
        applyConstraints();
    }

} // namespace rigging