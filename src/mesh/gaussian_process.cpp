#include "mdv/mesh/gaussian_process.hpp"

#include <stdexcept>
#include <vector>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/kernel.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {

struct InexactGaussianProcess::GpData {
    using ShortestPath = internal::CgalImpl::ShortestPath;
    PointVector               train_points;
    Eigen::MatrixXd           y_true;
    Eigen::MatrixXd           Kxx_inv;
    std::vector<ShortestPath> shpath_objs;

    GpData(const PointVector&     pts,
           const Eigen::MatrixXd& y,
           const double           noise,
           const double           ls) :
            train_points(pts), y_true(y) {
        if (y.rows() != pts.size()) {
            const auto msg = fmt::format(
                    "Ground truth rows ({}) differs from points size ({})",
                    y.rows(),
                    pts.size()
            );
            throw std::runtime_error(msg);
        }

        using internal::CgalGeodesicConstructor;

        // Construct shortest path objects
        const auto& mesh = pts[0].mesh();
        const auto& m    = internal::get_mesh_impl(pts[0].mesh());

        shpath_objs.reserve(train_points.size());
        for (const auto& pt : train_points) {
            shpath_objs.emplace_back(m);
            CgalGeodesicConstructor::set_source(shpath_objs.back(), pt);
        }
        const long n_pts = train_points.size();

        Eigen::MatrixXd distances = build_distance_matrix(train_points);
        assert(distances.rows() == distances.cols());
        assert(distances.rows() == n_pts);

        const Eigen::MatrixXd Kxx =
                MeshKernel::squared_exponential_from_matrix(distances, ls);

        Eigen::LLT<Eigen::MatrixXd> llt(Kxx);
        if (llt.info() != Eigen::Success)
            throw std::runtime_error("Covariance matrix is ill-defined");

        Kxx_inv = (noise * Eigen::MatrixXd::Identity(n_pts, n_pts) + Kxx).inverse();
    }

    Eigen::MatrixXd
    build_distance_matrix(const PointVector& test_points) {
        using internal::CgalGeodesicConstructor;

        assert(shpath_objs.size() == train_points.size());
        Eigen::MatrixXd distances(test_points.size(), train_points.size());

        std::vector<std::thread> threads;
        threads.reserve(distances.cols());

        auto column_processor = [this, &distances, &test_points](const long j) {
            for (long i = 0; i < distances.rows(); ++i) {
                const auto path = CgalGeodesicConstructor::construct_geodesic(
                        shpath_objs[j], test_points[i]
                );
                distances(i, j) = length(path);
            }
        };

        for (long j = 0; j < distances.cols(); ++j)
            threads.emplace_back(column_processor, j);
        for (auto& th : threads) th.join();
        return distances;
    }
};

InexactGaussianProcess::InexactGaussianProcess(
        const Mesh* mesh, const double lengthscale, double sigma_noise
) :
        _mesh(mesh), _ls(lengthscale), _sigma_noise(sigma_noise) {
    if (_ls <= 0.0) throw std::runtime_error("Lengthscale must be positive definite!");
    if (_sigma_noise < 0.0) throw std::runtime_error("Noise must be non-negative");
}

InexactGaussianProcess::~InexactGaussianProcess() {
    delete _data;
}

void
InexactGaussianProcess::train(
        const PointVector& in_points, const Eigen::MatrixXd& y_ref
) {
    for (const auto& pt : in_points) {
        if (&pt.mesh() != _mesh) throw std::runtime_error("Point not on same mesh!");
    }
    delete _data;
    _data = new GpData(in_points, y_ref, _sigma_noise, _ls);
}

Eigen::MatrixXd
InexactGaussianProcess::predict(const PointVector& test_points) {
    using internal::CgalGeodesicConstructor;

    if (_data == nullptr)
        throw std::runtime_error("Can't predict from uninitialised gaussian process!");

    Eigen::MatrixXd       distances = _data->build_distance_matrix(test_points);
    const Eigen::MatrixXd Kxp =
            MeshKernel::squared_exponential_from_matrix(distances, _ls);

    return Kxp * _data->Kxx_inv * _data->y_true;
}

}  // namespace mdv::mesh
