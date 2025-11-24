#ifndef MDV_MESH_TANGENT_VECTOR_HPP
#define MDV_MESH_TANGENT_VECTOR_HPP

#include <optional>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {

class TangentVector {
public:
    TangentVector(const Point& app_point, const Eigen::Vector3d& v);

    /**
     * @brief Constructs a tangent vector given its application point and the
     * "tip" position of the vector.
     */
    static TangentVector from_tip_position(
            const Point& application_point, const CartesianPoint& tip
    );

    static TangentVector unit_random(const Point& application_point);

    MDV_NODISCARD CartesianPoint tip() const noexcept;

    MDV_NODISCARD Eigen::Vector3d cartesian_vector() const noexcept;

    /**
     * @brief Scales the tangent vector length by the provided factor, i.e.
     *      v_new = factor * v
     *
     */
    void scale(const double& factor);

    /**
     * @brief Normalise the vector to make sure it has unitary norm (in cartesian
     * space).
     *
     */
    void normalise();

    /**
     * @brief Returns a (new) normalised tangent vector.
     *
     */
    TangentVector normalised() &;
    TangentVector normalised() &&;

    // clang-format off
    MDV_NODISCARD const Point&    application_point() const { return _pt; }

    // clang-format on

private:
    friend Point exponential_map(TangentVector, Geodesic*);

    Point _pt;
    Vec3d _vec;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_TANGENT_VECTOR_HPP
