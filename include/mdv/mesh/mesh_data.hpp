#ifndef MDV_MESH_DATA_HPP
#define MDV_MESH_DATA_HPP

#include <gsl/pointers>

#include "mdv/mesh/eigen_data.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh::internal {

class MeshData {
public:
    ~MeshData();
    MeshData()                           = default;
    MeshData(const MeshData&)            = default;
    MeshData(MeshData&&)                 = delete;
    MeshData& operator=(const MeshData&) = default;
    MeshData& operator=(MeshData&&)      = delete;

    MeshData(EigenData eigen_data, SpdLoggerPtr logger) :
            eigen_data(std::move(eigen_data)), logger(std::move(logger)) {}

    EigenData             eigen_data;
    gsl::owner<CgalImpl*> impl   = nullptr;
    SpdLoggerPtr          logger = nullptr;
};

}  // namespace mdv::mesh::internal


#endif  // MDV_MESH_DATA_HPP
