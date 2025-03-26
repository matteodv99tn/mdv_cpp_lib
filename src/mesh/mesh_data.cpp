#include "mdv/mesh/mesh_data.hpp"

#include <spdlog/spdlog.h>

#include "mdv/mesh/cgal_impl.hpp"

using mdv::mesh::internal::MeshData;

MeshData::~MeshData() {
    logger->debug("Releasing Cgal implementation pointer");
    delete impl;
}
