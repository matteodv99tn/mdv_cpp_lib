#ifndef MDV_MESH_TANGENT_VECTOR_HPP
#define MDV_MESH_TANGENT_VECTOR_HPP

#include <optional>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/index_element.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/uv_map.hpp"

namespace mdv::mesh {

class TangentVector : Point {
public:
    using UvCoord = Point::UvCoord;

    TangentVector(const Point& app_point, const UvCoord& uv) :
            Point(app_point), _uv(uv){};
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
     * @brief Trims a vector and return the "remainder" projected.
     *
     * Generally, a tangent vector could extend outside the boarder of the face.
     * This function virtually "cuts" the vector at the edge, and return the remainder
     * of the vector projected on the contiguous face based on conformal mappings.
     *
     * Note that if the vector was able to fit within the triangular face, it is left
     * unchanged.
     *
     */
    std::optional<TangentVector> trim();

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
    MDV_NODISCARD const UvCoord&  uv() const                { return _uv; }
    using Point::uv_map;
    using internal::MeshElement::logger;
    MDV_NODISCARD const Point&    application_point() const { return *this; }

    // clang-format on

private:
    UvCoord _uv;

    MDV_NODISCARD
    const UvMap::Transform&
    jac() const {
        return uv_map().forward_map_jacobian();
    }
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_TANGENT_VECTOR_HPP
