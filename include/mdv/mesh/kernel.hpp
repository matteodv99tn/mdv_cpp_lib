#ifndef MDV_MESH_KERNEL_HPP
#define MDV_MESH_KERNEL_HPP

#include <Eigen/Dense>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

class MeshKernel {
public:
    using PointVector = std::vector<Point>;

    MeshKernel(const Mesh& mesh);

    MDV_NODISCARD Eigen::MatrixXd evaluate_distance_matrix(const PointVector& pts) const;

    MDV_NODISCARD
    Eigen::MatrixXd evaluate_distance_matrix(
            const PointVector& pts1, const PointVector& pts2
    ) const;

private:
    const Mesh* _mesh;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_KERNEL_HPP
