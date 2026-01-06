#ifndef MDV_MESH_TANGENT_VECTOR_HPP
#define MDV_MESH_TANGENT_VECTOR_HPP

#include <cstdint>
#include <optional>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {

class TangentVector {
public:
    enum Type : std::uint8_t {
        INSIDE_FACE,
        ALONG_EDGE
    };

    TangentVector(const Point& app_point, const Eigen::Vector3d& v) :
            TangentVector(app_point, v, INSIDE_FACE) {};

    static TangentVector from_ambient_vector(
            const Point& application_point, const Eigen::Vector3d& vec
    );

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
    MDV_NODISCARD Type            type() const { return _type; }
    MDV_NODISCARD const HalfEdge& halfedge() const { return _he.value(); }

    // clang-format on

private:
    TangentVector(const Point& app_point, const Eigen::Vector3d& v, Type type);

    void validate_app_point_inside_face();
    void validate_app_point_on_edge();
    void validate_app_point_on_vertex();

    static TangentVector from_ambient_vector_on_edge(
            const Point::PointOnEdgeDescriptor& application_point,
            const Eigen::Vector3d&              vec
    );

    static TangentVector from_ambient_vector_on_vertex(
            const Point::PointOnVertexDescriptor& application_point,
            const Eigen::Vector3d&                vec
    );

    void fill_halfedge();

    friend Point exponential_map(TangentVector, Geodesic*);

    Point                   _pt;
    Vec3d                   _vec;
    Type                    _type;
    std::optional<HalfEdge> _he;
};

namespace internal {

    /*
     * Let v1 and v3 be vectors describing the outbound halfedges of a vertex, and v2
     * another vector in the plane given by v1 and v3. This function checks wether v2
     * lies in the interior of the triangular face or not.
     *
     * The function expects (and checks) that all vectors are coplanar.
     */
    bool vector_inside_triangle(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3);

    bool are_coplanar(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3);

}  // namespace internal


}  // namespace mdv::mesh


#endif  // MDV_MESH_TANGENT_VECTOR_HPP
