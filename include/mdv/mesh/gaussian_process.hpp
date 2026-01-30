#ifndef MDV_MESH_GAUSSIAN_PROCESS_HPP
#define MDV_MESH_GAUSSIAN_PROCESS_HPP

#include <Eigen/Dense>
#include <gsl/pointers>

#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

/**
 * @brief Gaussian process regression on a mesh using geodesic kernels.
 *
 * Designed for learning scalar or vector fields on surfaces where distances
 * are intrinsic to the mesh geometry.
 */
class InexactGaussianProcess {
public:
    using PointVector = std::vector<Point>;

    /**
     * @brief Constructs a GP model for a mesh.
     *
     * @param mesh Surface mesh.
     * @param lengthscale Kernel lengthscale.
     * @param sigma_noise Observation noise standard deviation.
     */
    InexactGaussianProcess(
            const Mesh* mesh, double lengthscale, double sigma_noise = 0.0
    );

    /**
     * @brief Destructor.
     */
    ~InexactGaussianProcess();

    /**
     * @brief Trains the GP on input points and target values.
     *
     * @param in_points Training points on the mesh.
     * @param y_ref Target values (rows aligned with points).
     */
    void train(const PointVector& in_points, const Eigen::MatrixXd& y_ref);

    /**
     * @brief Predicts output values at test points.
     *
     * @param test_points Query points on the mesh.
     * @return Predicted values.
     */
    Eigen::MatrixXd predict(const PointVector& test_points);


private:
    struct GpData;

    const Mesh*         _mesh        = nullptr;
    double              _ls          = 0.0;
    double              _sigma_noise = 0.0;
    gsl::owner<GpData*> _data        = nullptr;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_GAUSSIAN_PROCESS_HPP
