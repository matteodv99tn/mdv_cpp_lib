#ifndef MDV_MESH_DATA_HPP
#define MDV_MESH_DATA_HPP

#include <gsl/pointers>
#include <string>

#include "mdv/mesh/eigen_data.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh::internal {

class MeshData {
public:
    ~MeshData();
    MeshData()                = default;
    MeshData(const MeshData&) = delete;
    MeshData(MeshData&&);
    MeshData& operator=(const MeshData&) = delete;
    MeshData& operator=(MeshData&&);

    MeshData(EigenData eigen_data, Logger::SharedPtr logger) :
            eigen_data(std::move(eigen_data)), logger(std::move(logger)) {}

    EigenData             eigen_data;
    gsl::owner<CgalImpl*> impl   = nullptr;
    Logger::SharedPtr     logger = nullptr;
    std::string           name;
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_DATA_HPP
