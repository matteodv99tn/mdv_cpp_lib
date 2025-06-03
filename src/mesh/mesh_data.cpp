#include "mdv/mesh/mesh_data.hpp"

#include "mdv/mesh/cgal_impl.hpp"

using mdv::mesh::internal::MeshData;

MeshData::~MeshData() {
    // When MeshData gets moved, logger is set to nullptr and cannot be used
    if (logger)
        logger->debug("~MeshData() called at addr {}", static_cast<const void*>(this));
    if (impl) logger->debug("Releasing CGAL implementation pointer");
    delete impl;
}

MeshData::MeshData(MeshData&& other) :
        eigen_data(std::move(other.eigen_data)),
        logger(other.logger),
        impl(other.impl),
        name(std::move(other.name)) {
    other.impl = nullptr;
    this->logger->trace(
            "Moved MeshData from {} to {}",
            static_cast<const void*>(&other),
            static_cast<const void*>(this)
    );
}

MeshData&
MeshData::operator=(MeshData&& other) {
    this->eigen_data = std::move(eigen_data);
    this->impl       = other.impl;
    this->logger     = other.logger;
    this->name       = std::move(other.name);

    other.impl = nullptr;

    this->logger->trace(
            "Moved MeshData from {} to {}",
            static_cast<const void*>(&other),
            static_cast<const void*>(this)
    );
    return *this;
}
