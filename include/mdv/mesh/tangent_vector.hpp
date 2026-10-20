#ifndef MDV_MESH_TANGENT_VECTOR_HPP
#define MDV_MESH_TANGENT_VECTOR_HPP

#include <cstdint>
#include <optional>

#include "mdv/mesh/point.hpp"

namespace mdv::mesh {
/** @brief Self-contained ambient 3D tangent vector, with an explicit face frame. */
class TangentVector {
public:
    enum Type : std::uint8_t {
        INSIDE_FACE,
        ALONG_EDGE
    };

    TangentVector(const Mesh& mesh, const Point& p, const Vec3d& v) :
            TangentVector(mesh, p, v, INSIDE_FACE) {}

    /** @brief Construct using an explicitly selected incident face. */
    TangentVector(const Mesh& mesh, const Point& p, const Vec3d& v, FaceId preferred) :
            TangentVector(mesh, p, v, INSIDE_FACE, preferred) {}

    static TangentVector from_ambient_vector(const Mesh&, const Point&, const Vec3d&);
    static TangentVector from_tip_position(
            const Mesh&, const Point&, const CartesianPoint&
    );
    static TangentVector unit_random(const Mesh&, const Point&);

    MDV_NODISCARD const Point&
    application_point() const {
        return _pt;
    }

    MDV_NODISCARD FaceId
    face() const {
        return _f;
    }

    MDV_NODISCARD const Vec3d&
    cartesian_vector() const noexcept {
        return _vec;
    }

    MDV_NODISCARD Type
    type() const {
        return _type;
    }

    MDV_NODISCARD std::optional<HalfEdgeId>
                  halfedge() const {
        return _he;
    }

    void                        scale(double factor);
    void                        normalise();
    MDV_NODISCARD TangentVector normalised() const;

private:
    TangentVector(
            const Mesh&, Point, Vec3d, Type, std::optional<FaceId> = std::nullopt
    );
    void                 validate_app_point_inside_face(const Mesh&);
    void                 validate_app_point_on_edge(const Mesh&);
    void                 validate_app_point_on_vertex(const Mesh&);
    void                 fill_halfedge(const Mesh&);
    static TangentVector from_ambient_vector_on_edge(
            const Mesh&, const PointOnEdge&, const Vec3d&
    );
    static TangentVector from_ambient_vector_on_vertex(
            const Mesh&, const PointOnVertex&, const Vec3d&
    );
    Point                     _pt;
    FaceId                    _f;
    Vec3d                     _vec;
    Type                      _type;
    std::optional<HalfEdgeId> _he;
};

CartesianPoint tip(const Mesh&, const TangentVector&);

namespace internal {
    bool vector_inside_triangle(const Vec3d&, const Vec3d&, const Vec3d&);
    bool are_coplanar(const Vec3d&, const Vec3d&, const Vec3d&);
}  // namespace internal
}  // namespace mdv::mesh
#endif  // MDV_MESH_TANGENT_VECTOR_HPP
