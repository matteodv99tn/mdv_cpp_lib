#include "mdv/mesh/mesh_manager.hpp"

#include <cassert>
#include <fmt/format.h>
#include <stdexcept>

#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/logging.hpp"

using mdv::mesh::Mesh;
using mdv::mesh::MeshManager;

mdv::Logger::SharedPtr MeshManager::default_logger = get_default_logger();

void
MeshManager::insert_mesh(Mesh&& mesh, std::optional<std::string> name) {
    const std::string mesh_name = name.value_or(std::string(mesh.name()));
    _logger->info("Inserting new mesh with name '{}' into cache", mesh_name);

    if (_mesh_map.contains(mesh_name)) {
        _logger->error(
                "MeshManager: Trying to insert mesh named '{}' that already exists!",
                mesh_name
        );
        throw std::runtime_error("Inserting mesh with same name");
    }

    _mesh_map.insert(std::make_pair(mesh_name, std::move(mesh)));
    _logger->trace("Mesh correctly inserted!");
}

std::optional<MeshManager::MeshPtr>
MeshManager::get_mesh_by_name(const std::string& name) {
    _logger->info("Retrieving mesh with name '{}'", name);

    if (!_mesh_map.contains(name)) {
        _logger->warn("MeshManager: Unable to find mesh with name '{}'", name);
        return std::nullopt;
    }

    return &_mesh_map.at(name);
}

MeshManager::MeshPtr
MeshManager::get_mesh_from_file(const std::filesystem::path& file_path) {
    _logger->debug("Retrieving mesh from file {}", file_path.string());


    if (!_file_map.contains(file_path)) {
        auto mesh            = Mesh::from_file(file_path);
        _file_map[file_path] = std::string(mesh.name());
        insert_mesh(std::move(mesh));
        _logger->trace(
                "Mesh '{}' cached", file_path.string(), mesh.name()
        );
    }

    const std::string      mesh_name  = _file_map.at(file_path);
    std::optional<MeshPtr> maybe_mesh = get_mesh_by_name(mesh_name);
    if (!maybe_mesh.has_value()) {
        _logger->error(
                "MeshManager: mesh with file {} present in file map, but not "
                "present in mesh map!",
                file_path.string()
        );
        throw std::runtime_error("Unable to recover already loaded file");
    }

    return maybe_mesh.value();
}
