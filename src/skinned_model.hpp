#pragma once

#include <givr.h>
#include <optional>

namespace skinning {

	///////////////////////////////////////////////////////////
	// This ia a standard face list mesh model with bone     //
	// weights. The model file has information on the bone   //
	// lengths that are in an IMPLICIT rest orientation of   //
	// them lined in the x axis (default no rotation in the  //
	// SimpleArm). As such, this will need to be modified    //
	// for more complicated articulated structures that are  //
	// skinned. To complete the skinning, you need to modify //
	// the "skinnedVertexPositions" function to output the   //
	// posed position of each vertex (in the same order as   //
	// the corresponding std::vector). The main file will    //
	// also need to change to ensure the transforms passed   //
	// into "updateMesh", which calls						 //
	// "skinnedVertexPositions", are correct (render         // 
	// section).											 //
	///////////////////////////////////////////////////////////
	struct SkinnedModel {
		//Bone used in the model given by name (from file) and length
		struct Bone {
			std::string name;
			float length = 1.f;
		};
		//Face that indictes the vertices to connect
		struct Face {
			int va_id = -1;
			int vb_id = -1;
			int vc_id = -1;
		};
		//Vertex with a rest poition (rest pose) and a series of weights for relevent bones
		struct Vertex {
			glm::vec3 rest_position = { 0.f, 0.f, 0.f };
			//Weight for bone 
			struct BoneWeight {
				int bone_id = -1;
				float w = 0.f;
			};
			std::vector<BoneWeight> bone_weights; //Need to apply these weights in "skinnedVertexPositions"
		};

		//Call this once to initially create mesh
		givr::geometry::TriangleSoup makeMesh() const;
		//Update geometry for rest pose
		void updateMesh(givr::geometry::TriangleSoup& mesh) const;
		//Update geometry for pose
		void updateMesh(
			givr::geometry::TriangleSoup& mesh,
			//Assuming same bone ordering as the std::vector
			const std::vector<glm::mat4>& bone_rest_transformations,
			const std::vector<glm::mat4>& bone_posed_transformations
		) const;

		// TODO: Skinning Function (called by updateMesh(_,_,_)) to calculate the position of verices.
		// currently this only directly copies the rest position
		std::vector<glm::vec3> skinnedVertexPositions(
			//Assuming same bone ordering as the std::vector
			const std::vector<glm::mat4>& bone_rest_transformations
			, const std::vector<glm::mat4>& bone_posed_transformations
		) const;

		// Static Mesh loader. Doesnt use standard obj, but a custom format in a txt (a script for generating
		// this from blender will be provided)
		static std::optional<SkinnedModel>
			loadFromFile(std::string const& filepath);

		//Data
		std::vector<Bone> bones;
		std::vector<Face> faces;
		std::vector<Vertex> vertices;
	};
} // namespace skinning