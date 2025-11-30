#include "mdv/mesh/kernel.hpp"

#include <Eigen/Dense>
#include <thread>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {
namespace {

    using ShortestPath = internal::CgalImpl::ShortestPath;
    using internal::CgalGeodesicConstructor;

    void
    process_row(
            ShortestPath&                  shpath,
            const long                     i,  // row index
            const MeshKernel::PointVector& pts,
            Eigen::MatrixXd&               out
    ) {
        for (long j = 0; j < out.cols(); ++j) {
            const auto path =
                    CgalGeodesicConstructor::construct_geodesic(shpath, pts[j]);
            out(i, j) = length(path);
            assert(out(i, j) >= 0.0);
        }
    }

    Eigen::MatrixXd
    eval_distance_matrix(
            const Mesh&                    mesh,
            const MeshKernel::PointVector& pts1,
            const MeshKernel::PointVector& pts2
    ) {
        using internal::CgalImpl, internal::CgalGeodesicConstructor,
                internal::location_from_mesh_point;

        const auto      m = internal::get_mesh_impl(mesh);
        Eigen::MatrixXd res(pts1.size(), pts2.size());

        auto row_processor = [&pts1, &pts2, &res, &m](long i) {
            ShortestPath obj(m);
            const Point& target = pts1[i];
            CgalGeodesicConstructor::set_source(obj, target);
            process_row(obj, i, pts2, res);
        };

        std::vector<std::thread> threads;
        threads.reserve(res.rows());
        for (long i = 0; i < res.rows(); ++i) threads.emplace_back(row_processor, i);
        for (auto& th : threads) th.join();
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

Eigen::MatrixXd
MeshKernel::squared_exponential_from_matrix(
        const Eigen::MatrixXd& distance_matrix, const double lengthscale
) const {
    const double    lambda = 0.5 / (lengthscale * lengthscale);
    Eigen::MatrixXd res    = distance_matrix.unaryExpr([lambda](const double x) {
        return std::exp(-lambda * x * x);
    });
    return res;
}

Eigen::MatrixXd
MeshKernel::squared_exponential(
        const PointVector& pts1, const PointVector& pts2, const double lengthscale
) const {
    return squared_exponential_from_matrix(
            evaluate_distance_matrix(pts1, pts2), lengthscale
    );
}
}


}  // namespace mdv::mesh
