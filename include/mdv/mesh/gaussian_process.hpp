#ifndef MDV_MESH_GAUSSIAN_PROCESS_HPP
#define MDV_MESH_GAUSSIAN_PROCESS_HPP

#include <Eigen/Dense>
#include <gsl/pointers>

#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

class InexactGaussianProcess {
public:
    using PointVector = std::vector<Point>;

    InexactGaussianProcess(
            const Mesh* mesh, double lengthscale, double sigma_noise = 0.0
    );

    ~InexactGaussianProcess();

    void train(const PointVector& in_points, const Eigen::MatrixXd& y_ref);

    Eigen::MatrixXd predict(const PointVector& test_points);


private:
    struct GpData;

    const Mesh*         _mesh        = nullptr;
    const double        _ls          = 0.0;
    const double        _sigma_noise = 0.0;
    gsl::owner<GpData*> _data        = nullptr;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_GAUSSIAN_PROCESS_HPP
