#ifndef MDV_MESH_KERNEL_HPP
#define MDV_MESH_KERNEL_HPP

#include <Eigen/Dense>
#include <gsl/pointers>
#include <stdexcept>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

namespace internal {

    struct Data;

    /**
     * @brief Evaluates the squared exponential kernel on a distance matrix.
     *
     * @param d_matrix Distance matrix.
     * @param ls Lengthscale.
     * @return Kernel matrix.
     */
    Eigen::MatrixXd eval_sek(const Eigen::MatrixXd& d_matrix, double ls);

    /**
     * @brief Returns true if the matrix is positive definite.
     *
     * @param mat Matrix to test.
     * @return True if positive definite.
     */
    bool is_positive_definite(const Eigen::MatrixXd& mat);

}  // namespace internal

/**
 * @brief Base class for stationary kernels defined by distances.
 *
 * Used to build kernels on surfaces for learning and regression tasks.
 */
template <typename InputType>
struct StationaryDistanceKernel {
    using Input       = InputType;
    using InputVector = std::vector<Input>;
    using Output      = Eigen::MatrixXd;

    /**
     * @brief Virtual destructor.
     */
    virtual ~StationaryDistanceKernel() = default;

    /**
     * @brief Computes distance matrix between two point sets.
     *
     * @param pts1 First point set.
     * @param pts2 Second point set.
     * @return Distance matrix.
     */
    MDV_NODISCARD virtual Eigen::MatrixXd distance_matrix(
            const InputVector& pts1, const InputVector& pts2
    ) const = 0;

    /**
     * @brief Computes distance matrix for cached points.
     *
     * @param pts1 Cached point set.
     * @return Distance matrix.
     */
    MDV_NODISCARD virtual Eigen::MatrixXd
    distance_matrix(const InputVector& pts1) {
        (void)pts1;
        throw std::runtime_error(
                "StationaryDistanceKernel::distance_matrix not implemented for cached "
                "input"
        );
    }

    /**
     * @brief Returns true if the kernel supports caching.
     *
     * @return True if caching is supported.
     */
    MDV_NODISCARD virtual bool
    has_cache_support() const {
        return false;
    }

    /**
     * @brief Sets cached points for single-argument evaluations.
     *
     * @param pts1 Point set to cache.
     */
    virtual void
    set_points1(const InputVector& pts1) {
        (void)pts1;
        throw std::runtime_error("StationaryDistanceKernel does not support caching!");
    }

    /**
     * @brief Evaluates the squared exponential kernel between two point sets.
     *
     * @param pts1 First point set.
     * @param pts2 Second point set.
     * @param ls Lengthscale.
     * @return Kernel matrix.
     */
    Output
    operator()(
            const InputVector& pts1, const InputVector& pts2, const double ls
    ) const {
        return internal::eval_sek(distance_matrix(pts1, pts2), ls);
    };

    /**
     * @brief Evaluates the squared exponential kernel using cached points.
     *
     * @param pts2 Second point set.
     * @param ls Lengthscale.
     * @return Kernel matrix.
     */
    Output
    operator()(const InputVector& pts2, const double ls) {
        return internal::eval_sek(distance_matrix(pts2), ls);
    };
};

/**
 * @brief Exact geodesic kernel on a mesh.
 *
 * Computes geodesic distances between points on a surface and evaluates
 * stationary kernels, supporting learning methods that rely on intrinsic
 * geometry.
 */
class MeshKernel : public StationaryDistanceKernel<Point> {
public:
    /**
     * @brief Constructs a kernel bound to a mesh.
     *
     * @param mesh Surface mesh.
     */
    MeshKernel(const Mesh& mesh);

    /**
     * @brief Virtual destructor.
     */
    virtual ~MeshKernel() = default;

    /**
     * @brief Computes geodesic distance matrix between two point sets.
     *
     * @param pts1 First point set.
     * @param pts2 Second point set.
     * @return Distance matrix.
     */
    MDV_NODISCARD
    Eigen::MatrixXd distance_matrix(
            const InputVector& pts1, const InputVector& pts2
    ) const override;

    /**
     * @brief Estimates a maximum lengthscale that yields PD kernels.
     *
     * @param ls0 Initial lengthscale.
     * @param lsmax Maximum lengthscale to test.
     * @param num_points Points per test set.
     * @param num_tests Number of random test sets.
     * @param verbose Enables logging.
     * @return Estimated maximum lengthscale.
     */
    double find_max_lengthscale(
            double      ls0,
            double      lsmax      = 1.0,
            std::size_t num_points = 10,
            std::size_t num_tests  = 100,
            bool        verbose    = false
    );

    /**
     * @brief Estimates a maximum lengthscale for a fixed point set.
     *
     * @param pts Point set.
     * @param num_steps Number of search steps.
     * @return Estimated maximum lengthscale.
     */
    MDV_NODISCARD double find_pointset_max_lengthscale(
            const InputVector& pts, std::size_t num_steps = 30
    ) const;


protected:
    const Mesh* _mesh;
};

/**
 * @brief Cached, inexact geodesic kernel for repeated queries.
 *
 * This kernel is deemes "inexact" because if points are defined on half-edges, the
 * shortest path algorithm may give wrong geodesics in some cases (to be further
 * investigated). In most practical cases, the InexactMeshKernel is still the preferred
 * kernel for faster evaluation.
 */
class InexactMeshKernel : public MeshKernel {
public:
    /**
     * @brief Constructs an inexact kernel bound to a mesh.
     *
     * @param mesh Surface mesh.
     */
    InexactMeshKernel(const Mesh& mesh) : MeshKernel(mesh) {}

    /**
     * @brief Destructor.
     */
    ~InexactMeshKernel();

    /**
     * @brief Computes geodesic distance matrix between two point sets.
     *
     * @param pts1 First point set.
     * @param pts2 Second point set.
     * @return Distance matrix.
     */
    MDV_NODISCARD
    Eigen::MatrixXd distance_matrix(
            const InputVector& pts1, const InputVector& pts2
    ) const override;

    /**
     * @brief Computes distances from cached points to a new point set.
     *
     * @param pts2 Point set to compare against cached points.
     * @return Distance matrix.
     */
    MDV_NODISCARD Eigen::MatrixXd distance_matrix(const InputVector& pts2) override;

    /**
     * @brief Returns true (cache supported).
     *
     * @return True if caching is supported.
     */
    MDV_NODISCARD bool
    has_cache_support() const override {
        return true;
    }

    /**
     * @brief Sets the cached points for single-argument evaluation.
     *
     * @param pts1 Point set to cache.
     */
    void set_points1(const InputVector& pts1) override;

    Eigen::MatrixXd evaluate_single(const InputVector& pts2, double ls);

private:
    using Data = internal::Data;

    gsl::owner<Data*> _data = nullptr;
};

/**
 * @brief Finds a maximum lengthscale for a fixed distance matrix.
 *
 * @param dist_matrix Distance matrix.
 * @param num_steps Number of search steps.
 * @param ls0 Initial lengthscale.
 * @return Estimated maximum lengthscale.
 */
double find_matrix_max_lengthscale(
        const Eigen::MatrixXd& dist_matrix, std::size_t num_steps = 30, double ls0 = 0.1
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_KERNEL_HPP
