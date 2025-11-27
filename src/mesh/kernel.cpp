#include "mdv/mesh/kernel.hpp"

#include <Eigen/Dense>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {
namespace {

    Eigen::MatrixXd
    eval_distance_matrix(
            const Mesh&                    mesh,
            const MeshKernel::PointVector& pts1,
            const MeshKernel::PointVector& pts2
    ) {
        using internal::CgalImpl, internal::CgalGeodesicConstructor,
                internal::location_from_mesh_point;
        using ShortestPath = internal::CgalImpl::ShortestPath;

        const auto      m = internal::get_mesh_impl(mesh);
        Eigen::MatrixXd res(pts1.size(), pts2.size());

        for (long i = 0; i < res.rows(); ++i) {
            ShortestPath obj(m);
            CgalGeodesicConstructor::set_source(obj, pts1[i]);

            for (long j = 0; j < res.cols(); ++j) {
                const auto path =
                        CgalGeodesicConstructor::construct_geodesic(obj, pts2[j]);
                res(i, j) = length(path);
                assert(res(i, j) >= 0.0);
            }
        }
        return res;
    }
}  // namespace

MeshKernel::MeshKernel(const Mesh& mesh) : _mesh(&mesh) {
}

Eigen::MatrixXd
MeshKernel::evaluate_distance_matrix(const PointVector& pts) const {
    return evaluate_distance_matrix(pts, pts);
}

Eigen::MatrixXd
MeshKernel::evaluate_distance_matrix(
        const PointVector& pts1, const PointVector& pts2
) const {
    const Eigen::MatrixXd D12 = eval_distance_matrix(*_mesh, pts1, pts2);
    const Eigen::MatrixXd D21 = eval_distance_matrix(*_mesh, pts2, pts1);
    assert(D12.cols() == D21.rows() && D12.rows() == D21.cols());

    const long      nr = D12.rows();
    const long      nc = D12.cols();
    Eigen::MatrixXd res(nr, nc);
    for (long i = 0; i < nr; ++i) {
        for (long j = 0; j < nc; ++j) { res(i, j) = std::min(D12(i, j), D21(j, i)); }
    }
    return res;
}


}  // namespace mdv::mesh
