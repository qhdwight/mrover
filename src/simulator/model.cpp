#include "simulator.hpp"

namespace mrover {

    using namespace std::literals;

    Model::Model(std::string_view uri) {
        // Note(quintin):
        // Ideally we would use glTF as it is open-source (unlike FBX) and has a binary format (unlike OBJ and STL)
        // However I could not get Blender to export the textures by filename
        // The only option was to embed them, but that results in needing to decode the data and duplicating it across models
        if (!uri.ends_with("fbx")) {
            ROS_WARN_STREAM(std::format("Model importer has only been tested with the FBX file format: {}", uri));
        }

        // assimp's scene import is slow on the larger rover models, so we load it in a separate thread
        asyncMeshesLoader = std::async(std::launch::async, [uri] {
            Assimp::Importer importer;
            importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE); // Drop points and lines

            // aiScene const* scene = importer.ReadFile(uri.data(),aiProcessPreset_TargetRealtime_MaxQuality);
            aiScene const* scene = importer.ReadFile(uriToPath(uri), aiProcessPreset_TargetRealtime_Quality);
            if (!scene) {
                throw std::runtime_error{std::format("Scene import error: {} on path: {}", importer.GetErrorString(), uri)};
            }
            ROS_INFO_STREAM(std::format("Loaded scene: {} with mesh count: {}", uri, scene->mNumMeshes));
            if (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) throw std::runtime_error{std::format("Incomplete asset: {}", uri)};

            std::vector<Mesh> meshes;
            meshes.reserve(scene->mNumMeshes);

            for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
                aiMesh const* mesh = scene->mMeshes[meshIndex];

                if (!mesh->HasNormals()) throw std::invalid_argument{std::format("Mesh #{} has no normals", meshIndex)};
                if (!mesh->HasTextureCoords(0)) throw std::invalid_argument{std::format("Mesh #{} has no texture coordinates", meshIndex)};

                auto& [_vao, vertices, normals, uvs, indices, texture] = meshes.emplace_back();

                Eigen::Matrix3f assimpToRos;
                if (uri.ends_with("glb") || uri.ends_with("gltf")) {
                    // This importer has only been tested with Blender exporting to glTF
                    // Blender must be using ROS's coordinate system: +x forward, +y left, +z up
                    assimpToRos << 1, 0, 0,
                            0, 0, -1,
                            0, 1, 0;
                } else {
                    assimpToRos = Eigen::Matrix3f::Identity();
                }

                assert(mesh->HasPositions());
                vertices.data.resize(mesh->mNumVertices);
                std::for_each(std::execution::par, mesh->mVertices, mesh->mVertices + mesh->mNumVertices, [&](aiVector3D const& vertex) {
                    std::size_t vertexIndex = &vertex - mesh->mVertices;
                    vertices.data[vertexIndex] = assimpToRos * Eigen::Vector3f{vertex.x, vertex.y, vertex.z};
                });

                indices.data.resize(mesh->mNumFaces * 3);
                for (uint faceIndex = 0; faceIndex < mesh->mNumFaces; ++faceIndex) {
                    aiFace const& face = mesh->mFaces[faceIndex];
                    assert(face.mNumIndices == 3);

                    indices.data[faceIndex * 3 + 0] = face.mIndices[0];
                    indices.data[faceIndex * 3 + 1] = face.mIndices[1];
                    indices.data[faceIndex * 3 + 2] = face.mIndices[2];
                }

                assert(mesh->HasNormals());
                normals.data.resize(mesh->mNumVertices);
                std::for_each(std::execution::par, mesh->mNormals, mesh->mNormals + mesh->mNumVertices, [&](aiVector3D const& normal) {
                    std::size_t normalIndex = &normal - mesh->mNormals;
                    normals.data[normalIndex] = assimpToRos * Eigen::Vector3f{normal.x, normal.y, normal.z};
                });

                assert(mesh->HasTextureCoords(0));
                uvs.data.resize(mesh->mNumVertices);
                std::for_each(std::execution::par, mesh->mTextureCoords[0], mesh->mTextureCoords[0] + mesh->mNumVertices, [&](aiVector3D const& uv) {
                    std::size_t uvIndex = &uv - mesh->mTextureCoords[0];
                    uvs.data[uvIndex] = {uv.x, uv.y};
                });

                if (aiMaterial const* material = scene->mMaterials[mesh->mMaterialIndex]) {
                    if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
                        if (aiString path; material->GetTexture(aiTextureType_DIFFUSE, 0, &path) == AI_SUCCESS) {
                            texture.data = readTexture(path.C_Str());
                        }
                    }
                    aiString name;
                    material->Get(AI_MATKEY_NAME, name);
                    ROS_INFO_STREAM(std::format("\tLoaded material: {}", name.C_Str()));
                }

                ROS_INFO_STREAM(std::format("\tLoaded mesh: #{} with {} vertices and {} faces", meshIndex, mesh->mNumVertices, mesh->mNumFaces));
            }

            return meshes;
        });
    }

    auto Model::waitMeshes() -> void {
        if (!asyncMeshesLoader.valid()) return;

        meshes = asyncMeshesLoader.get();
    }

    auto Model::areMeshesReady() -> bool {
        if (!asyncMeshesLoader.valid()) return true;

        if (asyncMeshesLoader.wait_for(0ms) == std::future_status::ready) {
            waitMeshes();
            return true;
        }
        return false;
    }

    Model::~Model() {
        waitMeshes();

        for (Mesh const& mesh: meshes) {
            if (mesh.vao == GL_INVALID_HANDLE) continue;
            glDeleteVertexArrays(1, &mesh.vao);
        }
    }
} // namespace mrover
