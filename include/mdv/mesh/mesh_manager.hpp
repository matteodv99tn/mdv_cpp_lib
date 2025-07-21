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

class MeshManager {
public:
    using MeshPtr = gsl::not_null<Mesh*>;

    ~MeshManager();

    void insert_mesh(
            gsl::owner<Mesh*> mesh, std::optional<std::string> name = std::nullopt
    );

    std::optional<MeshPtr> get_mesh_by_name(const std::string& name);

    MeshPtr get_mesh_from_file(const std::filesystem::path& file_path);


    static Logger::SharedPtr default_logger;

private:
    using FileMap = std::map<std::filesystem::path, std::string>;
    using MeshMap = std::map<std::string, gsl::owner<Mesh*>>;

    FileMap _file_map;
    MeshMap _mesh_map;

    Logger::SharedPtr _logger = default_logger;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_MANAGER_HPP
