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

    /*
     * Evaluates the squared exponential kernel on a given distance matrix
     */
    Eigen::MatrixXd eval_sek(const Eigen::MatrixXd& d_matrix, double ls);

    bool is_positive_definite(const Eigen::MatrixXd& mat);

}  // namespace internal

template <typename InputType>
struct StationaryDistanceKernel {
    using Input       = InputType;
    using InputVector = std::vector<Input>;
    using Output      = Eigen::MatrixXd;

    virtual ~StationaryDistanceKernel() = default;

    MDV_NODISCARD virtual Eigen::MatrixXd distance_matrix(
            const InputVector& pts1, const InputVector& pts2
    ) const = 0;

    MDV_NODISCARD virtual Eigen::MatrixXd
    distance_matrix(const InputVector& /*pts1*/) {
        throw std::runtime_error(
                "StationaryDistanceKernel::distance_matrix not implemented for cached "
                "input"
        );
    }

    MDV_NODISCARD virtual bool
    has_cache_support() const {
        return false;
    }

    virtual void
    set_points1(const InputVector& /* pts1 */) {
        throw std::runtime_error("StationaryDistanceKernel does not support caching!");
    }

    Output
    operator()(
            const InputVector& pts1, const InputVector& pts2, const double ls
    ) const {
        return internal::eval_sek(distance_matrix(pts1, pts2), ls);
    };

    Output
    operator()(const InputVector& pts2, const double ls) {
        return internal::eval_sek(distance_matrix(pts2), ls);
    };
};

class MeshKernel : public StationaryDistanceKernel<Point> {
public:
    MeshKernel(const Mesh& mesh);

    virtual ~MeshKernel() = default;

    MDV_NODISCARD
    Eigen::MatrixXd distance_matrix(
            const InputVector& pts1, const InputVector& pts2
    ) const override;

    double find_max_lengthscale(
            double      ls0,
            double      lsmax      = 1.0,
            std::size_t num_points = 10,
            std::size_t num_tests  = 100,
            bool        verbose    = false
    );

    MDV_NODISCARD double find_pointset_max_lengthscale(
            const InputVector& pts, std::size_t num_steps = 30
    ) const;


protected:
    const Mesh* _mesh;
};

class InexactMeshKernel : public MeshKernel {
public:
    InexactMeshKernel(const Mesh& mesh) : MeshKernel(mesh) {}

    ~InexactMeshKernel();

    MDV_NODISCARD
    Eigen::MatrixXd distance_matrix(
            const InputVector& pts1, const InputVector& pts2
    ) const override;

    MDV_NODISCARD Eigen::MatrixXd distance_matrix(const InputVector& pts2) override;

    MDV_NODISCARD bool
    has_cache_support() const override {
        return true;
    }

    void set_points1(const InputVector& pts1) override;

private:
    using Data = internal::Data;

    gsl::owner<Data*> _data = nullptr;
};

double find_matrix_max_lengthscale(
        const Eigen::MatrixXd& dist_matrix, std::size_t num_steps = 30, double ls0 = 0.1
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_KERNEL_HPP
