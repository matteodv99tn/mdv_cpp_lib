#ifndef MDV_MESH_TANGENT_VECTOR_HPP
#define MDV_MESH_TANGENT_VECTOR_HPP

#include <cstdint>
#include <optional>

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/point.hpp"

namespace mdv::mesh {

/**
 * @brief Tangent vector applied at a point on a mesh.
 *
 * Tangent vectors are the fundamental representation of directions for
 * geodesic integration, logarithmic/exponential maps, and policy learning on
 * surfaces.
 */
class TangentVector {
public:
    /**
     * @brief Tangent vector type relative to local geometry.
     */
    enum Type : std::uint8_t {
        INSIDE_FACE,
        ALONG_EDGE
    };

    /**
     * @brief Constructs a tangent vector inside the face of the application point.
     *
     * @param app_point Application point on the mesh.
     * @param v Cartesian vector in 3D.
     */
    TangentVector(const Point& app_point, const Eigen::Vector3d& v) :
            TangentVector(app_point, v, INSIDE_FACE) {};

    /**
     * @brief Projects a 3D vector onto the surface tangent space.
     *
     * @param application_point Application point on the mesh.
     * @param vec Cartesian vector in 3D.
     * @return Tangent vector on the mesh.
     */
    static TangentVector from_ambient_vector(
            const Point& application_point, const Eigen::Vector3d& vec
    );

    /**
     * @brief Constructs a tangent vector from a tip position in 3D.
     *
     * @param application_point Application point on the mesh.
     * @param tip Tip position in 3D.
     * @return Tangent vector on the mesh.
     */
    static TangentVector from_tip_position(
            const Point& application_point, const CartesianPoint& tip
    );

    /**
     * @brief Random unit tangent vector on a face.
     *
     * @param application_point Application point inside a face.
     * @return Unit tangent vector.
     */
    static TangentVector unit_random(const Point& application_point);

    /**
     * @brief Tip position (application point + vector).
     *
     * @return Tip position in 3D.
     */
    MDV_NODISCARD CartesianPoint tip() const noexcept;

    /**
     * @brief Cartesian vector in ambient 3D space.
     *
     * @return Cartesian vector.
     */
    MDV_NODISCARD Eigen::Vector3d cartesian_vector() const noexcept;

    /**
     * @brief Scales the vector magnitude by a factor.
     *
     * @param factor Scaling factor.
     */
    void scale(const double& factor);

    /**
     * @brief Normalizes the vector to unit length (Cartesian).
     */
    void normalise();

    /**
     * @brief Returns a normalized copy.
     *
     * @return Normalized tangent vector.
     */
    TangentVector normalised() &;
    TangentVector normalised() &&;

    /**
     * @brief Application point on the mesh.
     *
     * @return Application point.
     */
    MDV_NODISCARD const Point&
    application_point() const {
        return _pt;
    }

    /**
     * @brief Tangent vector type.
     *
     * @return Tangent vector type.
     */
    MDV_NODISCARD Type
    type() const {
        return _type;
    }

    /**
     * @brief Associated half-edge for edge/vertex cases.
     *
     * @return Half-edge reference.
     */
    MDV_NODISCARD const HalfEdge&
    halfedge() const {
        return _he.value();
    }

private:
    /**
     * @brief Constructs a tangent vector with explicit type.
     *
     * @param app_point Application point on the mesh.
     * @param v Cartesian vector in 3D.
     * @param type Tangent vector type.
     */
    TangentVector(const Point& app_point, const Eigen::Vector3d& v, Type type);

    /**
     * @brief Validates a vector applied inside a face.
     */
    void validate_app_point_inside_face();

    /**
     * @brief Validates a vector applied on an edge.
     */
    void validate_app_point_on_edge();

    /**
     * @brief Validates a vector applied on a vertex.
     */
    void validate_app_point_on_vertex();

    /**
     * @brief Projects an ambient vector at an edge point.
     *
     * @param application_point Edge descriptor.
     * @param vec Cartesian vector in 3D.
     * @return Tangent vector on the mesh.
     */
    static TangentVector from_ambient_vector_on_edge(
            const Point::PointOnEdgeDescriptor& application_point,
            const Eigen::Vector3d&              vec
    );

    /**
     * @brief Projects an ambient vector at a vertex point.
     *
     * @param application_point Vertex descriptor.
     * @param vec Cartesian vector in 3D.
     * @return Tangent vector on the mesh.
     */
    static TangentVector from_ambient_vector_on_vertex(
            const Point::PointOnVertexDescriptor& application_point,
            const Eigen::Vector3d&                vec
    );

    /**
     * @brief Initializes the half-edge associated with the application point.
     */
    void fill_halfedge();

    friend Point exponential_map(TangentVector, Geodesic*);

    Point                   _pt;
    Vec3d                   _vec;
    Type                    _type;
    std::optional<HalfEdge> _he;
};

namespace internal {

    /**
     * @brief Checks whether v2 lies inside the triangle spanned by v1 and v3.
     *
     * All vectors are assumed coplanar and represent directions emanating from a
     * vertex along adjacent half-edges.
     *
     * @param v1 First edge direction.
     * @param v2 Candidate direction.
     * @param v3 Second edge direction.
     * @return True if v2 is inside the triangle spanned by v1 and v3.
     */
    bool vector_inside_triangle(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3);

    /**
     * @brief Returns true if the three vectors are coplanar.
     *
     * @param v1 First vector.
     * @param v2 Second vector.
     * @param v3 Third vector.
     * @return True if coplanar.
     */
    bool are_coplanar(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3);

}  // namespace internal


}  // namespace mdv::mesh


#endif  // MDV_MESH_TANGENT_VECTOR_HPP
