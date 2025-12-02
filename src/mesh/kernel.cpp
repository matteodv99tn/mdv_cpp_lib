#include "mdv/mesh/kernel.hpp"

#include <Eigen/Core>
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

    bool
    is_positive_definite(const Eigen::MatrixXd& mat) {
        // Check if it is possible to perform a valid Cholesky decomposition
        Eigen::LLT<Eigen::MatrixXd> llt(mat);
        return llt.info() == Eigen::Success;
    }

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

double
MeshKernel::find_max_lengthscale(
        const double      ls0,
        const double      lsmax,
        const std::size_t num_points,
        const std::size_t num_tests,
        const bool        verbose
) {
    assert(_mesh);
    if (verbose) {
        std::cout << "Finding maximum lengthscale for mesh " << _mesh->name() << "\n";
        std::cout << "Number of random testing conditions: " << num_tests << "\n";
        std::cout << "Number of points per testing condition: " << num_points << "\n";
    }

    std::vector<Eigen::MatrixXd> dist_matrices;
    dist_matrices.reserve(num_tests);
    for (std::size_t i = 0; i < num_tests; ++i) {
        std::vector<mdv::mesh::Point> points;
        points.reserve(num_points);
        for (std::size_t j = 0; j < num_points; ++j)
            points.emplace_back(mdv::mesh::Point::random(*_mesh));

        dist_matrices.emplace_back(evaluate_distance_matrix(points));
    }

    double ls_min = 0.0;
    double ls_max = lsmax;
    double ls     = ls0;

    if (verbose) std::cout << "Starting search\n";
    for (std::size_t k = 0; k < 50; ++k) {
        ls = 0.5 * (ls_min + ls_max);

        bool is_pd = true;
        for (std::size_t i = 0; i < dist_matrices.size(); ++i) {
            const Eigen::MatrixXd ker =
                    squared_exponential_from_matrix(dist_matrices[i], ls);
            if (!is_positive_definite(ker)) {
                // std::cout << "Test #" << i << " yields negative covariance\n";
                is_pd = false;
                break;
            }
        }

        if (is_pd) ls_min = ls;
        else ls_max = ls;
    }

    return ls_min;
}


}  // namespace mdv::mesh
