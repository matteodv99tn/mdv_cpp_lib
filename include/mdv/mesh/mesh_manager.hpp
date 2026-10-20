#ifndef MDV_MESH_MANAGER_HPP
#define MDV_MESH_MANAGER_HPP

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh {

/**
 * @brief Mesh cache and lifetime manager.
 *
 * Provides ownership and lookup for meshes loaded from file, avoiding repeated
 * disk I/O and CGAL setup when training or evaluating policies on surfaces.
 */
class MeshManager {
public:
    using MeshPtr = std::shared_ptr<Mesh>;

    /**
     * @brief Default logger used by mesh managers.
     */
    static Logger::SharedPtr default_logger;

    /**
     * @brief Destructor; releases the cache's shared ownership of meshes.
     */
    ~MeshManager();

    /**
     * @brief Inserts a mesh into the cache.
     *
     * @param mesh Owned mesh pointer.
     * @param name Optional name override.
     */
    void insert_mesh(
            MeshPtr mesh, const std::optional<std::string>& name = std::nullopt
    );

    /**
     * @brief Retrieves a mesh by name if present.
     *
     * @param name Mesh name.
     * @return Optional shared mesh owner, valid independently of the manager.
     */
    std::optional<MeshPtr> get_mesh_by_name(const std::string& name);

    /**
     * @brief Loads and caches a mesh by file path.
     *
     * @param file_path Path to mesh file.
     * @return Shared mesh owner, valid independently of the manager.
     */
    MeshPtr get_mesh_from_file(const std::filesystem::path& file_path);


private:
    using FileMap = std::map<std::filesystem::path, std::string>;
    using MeshMap = std::map<std::string, MeshPtr>;

    FileMap _file_map;
    MeshMap _mesh_map;

    Logger::SharedPtr _logger = default_logger;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_MANAGER_HPP
