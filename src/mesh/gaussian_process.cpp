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
    PointVector       train_points;
    Eigen::MatrixXd   y_true;
    Eigen::MatrixXd   Kxx_inv;
    InexactMeshKernel kernel;

    GpData(const PointVector&     pts,
           const Eigen::MatrixXd& y,
           const double           noise,
           const double           ls) :
            train_points(pts), y_true(y), kernel(pts[0].mesh()) {
        if (y.rows() != pts.size()) {
            const auto msg = fmt::format(
                    "Ground truth rows ({}) differs from points size ({})",
                    y.rows(),
                    pts.size()
            );
            throw std::runtime_error(msg);
        }

        const long n_pts = train_points.size();

        kernel.set_points1(train_points);
        const Eigen::MatrixXd Kxx = kernel(train_points, ls);
        assert(Kxx.rows() == Kxx.cols());
        assert(Kxx.rows() == n_pts);

        if (!internal::is_positive_definite(Kxx))
            throw std::runtime_error("Covariance matrix is ill-defined");

        Kxx_inv = (noise * Eigen::MatrixXd::Identity(n_pts, n_pts) + Kxx).inverse();
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

    const Eigen::MatrixXd Kxp = (_data->kernel)(test_points, _ls).transpose();

    return Kxp * _data->Kxx_inv * _data->y_true;
}

}  // namespace mdv::mesh
