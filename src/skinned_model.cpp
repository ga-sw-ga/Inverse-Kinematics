#include "skinned_model.hpp"

namespace skinning {

	givr::geometry::TriangleSoup SkinnedModel::makeMesh() const {
		givr::geometry::TriangleSoup mesh;
		updateMesh(mesh); //Default rest geometry
		return mesh;
	}

	void SkinnedModel::updateMesh(givr::geometry::TriangleSoup& mesh) const {	
		mesh.triangles().clear();
		for (const Face& f : faces) { //Uses the rest pose for all vertices
			const Vertex& a = vertices[f.va_id];
			const Vertex& b = vertices[f.vb_id];
			const Vertex& c = vertices[f.vc_id];
			mesh.push_back(
				givr::geometry::Triangle(
					givr::geometry::Point1(a.rest_position),
					givr::geometry::Point2(b.rest_position),
					givr::geometry::Point3(c.rest_position)
				)
			);
		};
	}

	void SkinnedModel::updateMesh(
		givr::geometry::TriangleSoup& mesh,
		const std::vector<glm::mat4>& bone_rest_transformations,
		const std::vector<glm::mat4>& bone_posed_transformations
	) const {
		mesh.triangles().clear();
		//Caculates the posed positions (currently this does nothing but a copy of the rest pose)
		std::vector<glm::vec3> transformed_position = skinnedVertexPositions(
			bone_rest_transformations
			, bone_posed_transformations
		);
		for (const Face& f : faces) {
			const glm::vec3& a = transformed_position[f.va_id];
			const glm::vec3& b = transformed_position[f.vb_id];
			const glm::vec3& c = transformed_position[f.vc_id];
			mesh.push_back(
				givr::geometry::Triangle(
					givr::geometry::Point1(a),
					givr::geometry::Point2(b),
					givr::geometry::Point3(c)
				)
			);
		};
	}

    std::vector<glm::vec3> SkinnedModel::skinnedVertexPositions(
            const std::vector<glm::mat4>& bone_rest_transformations,
            const std::vector<glm::mat4>& bone_posed_transformations
    ) const {
        std::vector<glm::vec3> ret;

        // Apply linear blend skinning using the rest and pose transformations
        ret.reserve(vertices.size());
        for (const Vertex& v : vertices) {
            glm::vec3 skinned_position(0.0f);

            for (const Vertex::BoneWeight& bw : v.bone_weights) {
                const glm::mat4& rest_transform = bone_rest_transformations[bw.bone_id];
                const glm::mat4& posed_transform = bone_posed_transformations[bw.bone_id];

                // Calculate the skinned position using linear blend skinning
                skinned_position += bw.w * glm::vec3(posed_transform * glm::inverse(rest_transform) * glm::vec4(v.rest_position, 1.0f));
            }

            ret.push_back(skinned_position);
        }

        return ret;
    }


    std::optional<SkinnedModel> SkinnedModel::loadFromFile(std::string const& filepath) {
				SkinnedModel model;

		std::ifstream file(filepath);
		if (!file) return {};

		size_t n_vertices;
		file >> n_vertices;
		model.vertices.resize(n_vertices);
		for (Vertex& v : model.vertices)
			file >> v.rest_position.x >> v.rest_position.y >> v.rest_position.z;

		size_t n_faces;
		file >> n_faces;
		model.faces.resize(n_faces);
		for (Face& f : model.faces)
			file >> f.va_id >> f.vb_id >> f.vc_id;

		size_t n_bones;
		file >> n_bones;
		model.bones.resize(n_bones);
		for (size_t bone_id = 0; bone_id < n_bones; bone_id++)
			file >> model.bones[bone_id].name >> model.bones[bone_id].length;

		// load weights
		for (size_t bone_id = 0; bone_id < model.bones.size(); bone_id++) {
			size_t n_bone_vert_weights;
			file >> n_bone_vert_weights;
			for (int index = 0; index < n_bone_vert_weights; index++) {
				size_t vertex_id;
				file >> vertex_id;
				auto& vertex_weights = model.vertices[vertex_id].bone_weights;

				float w;
				file >> w;

				std::string name;
				file >> name; // not used, need to be clear from the buffer though

				Vertex::BoneWeight bw;
				bw.w = w;
				bw.bone_id = bone_id;

				vertex_weights.push_back(bw);
			}
		}

		return std::optional<SkinnedModel>{model};
	}

} // namespace skinning