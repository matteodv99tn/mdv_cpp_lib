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

    MDV_NODISCARD Eigen::MatrixXd evaluate_distance_matrix(
            const PointVector& pts
    ) const;

    MDV_NODISCARD
    Eigen::MatrixXd evaluate_distance_matrix(
            const PointVector& pts1, const PointVector& pts2
    ) const;

    MDV_NODISCARD
    Eigen::MatrixXd squared_exponential_from_matrix(
            const Eigen::MatrixXd& distance_matrix, const double lengthscale
    ) const;

    MDV_NODISCARD
    Eigen::MatrixXd squared_exponential(
            const PointVector& pts1, const PointVector& pts2, const double lengthscale
    ) const;

    double find_max_lengthscale(
            double      ls0,
            double      lsmax      = 1.0,
            std::size_t num_points = 10,
            std::size_t num_tests  = 100,
            bool        verbose    = false
    );

    double find_pointset_max_lengthscale(
            const PointVector& pts, std::size_t num_steps = 30
    );

private:
    const Mesh* _mesh;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_KERNEL_HPP
