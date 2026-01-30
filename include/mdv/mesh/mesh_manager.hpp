#ifndef MDV_MESH_MANAGER_HPP
#define MDV_MESH_MANAGER_HPP

#include <filesystem>
#include <gsl/pointers>
#include <map>
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
    using MeshPtr = gsl::not_null<Mesh*>;

    /**
     * @brief Default logger used by mesh managers.
     */
    static Logger::SharedPtr default_logger;

    /**
     * @brief Destructor; releases owned meshes.
     */
    ~MeshManager();

    /**
     * @brief Inserts a mesh into the cache.
     *
     * @param mesh Owned mesh pointer.
     * @param name Optional name override.
     */
    void insert_mesh(
            gsl::owner<Mesh*> mesh, std::optional<std::string> name = std::nullopt
    );

    /**
     * @brief Retrieves a mesh by name if present.
     *
     * @param name Mesh name.
     * @return Optional mesh pointer.
     */
    std::optional<MeshPtr> get_mesh_by_name(const std::string& name);

    /**
     * @brief Loads and caches a mesh by file path.
     *
     * @param file_path Path to mesh file.
     * @return Mesh pointer.
     */
    MeshPtr get_mesh_from_file(const std::filesystem::path& file_path);


private:
    using FileMap = std::map<std::filesystem::path, std::string>;
    using MeshMap = std::map<std::string, gsl::owner<Mesh*>>;

    FileMap _file_map;
    MeshMap _mesh_map;

    Logger::SharedPtr _logger = default_logger;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_MANAGER_HPP
