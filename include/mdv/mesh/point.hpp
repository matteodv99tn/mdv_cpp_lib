#ifndef MDV_MESH_POINT_HPP
#define MDV_MESH_POINT_HPP

#include <array>
#include <string>
#include <type_traits>
#include <variant>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/fwd.hpp"

namespace mdv::mesh {

struct PointOnVertex {
    VertexId v;
    bool     operator==(const PointOnVertex&) const = default;
};

struct PointOnEdge {
    HalfEdgeId         he;
    double             t;
    static PointOnEdge random(const Mesh&);
    bool               operator==(const PointOnEdge&) const = default;
};

struct PointInFace {
    FaceId                f;
    std::array<double, 3> bary;

    /** @brief Read-only view borrowing this descriptor's storage. */
    MDV_NODISCARD Eigen::Map<const Vec3d>
                  coords() const& noexcept {
        return Eigen::Map<const Vec3d>{bary.data()};
    }

    MDV_NODISCARD Eigen::Map<const Vec3d> coords() const&& = delete;
    bool operator==(const PointInFace&) const              = default;
};

/** @brief Mesh-free surface location. Topology edits may invalidate its handles.
 * All geometry operations require the corresponding mesh and a defined point.
 */
class Point {
public:
    using Descriptor = std::variant<PointOnVertex, PointOnEdge, PointInFace>;
    Point()          = default;
    Point(PointOnVertex d);
    Point(PointOnEdge d);
    Point(PointInFace d);
    Point(const Mesh&, FaceId, Vec3d bary, bool classify_boundary = true);
    MDV_NODISCARD bool is_undefined() const noexcept;

    MDV_NODISCARD const Descriptor&
    descriptor() const noexcept {
        return _d;
    }

    template <typename T>
    const T*
    get_as() const noexcept {
        return std::get_if<T>(&_d);
    }

    bool operator==(const Point&) const = default;

private:
    Descriptor _d;
};

static_assert(std::is_trivially_copyable_v<Point>);

CartesianPoint position(const Mesh&, const Point&);
FaceId         face(const Mesh&, const Point&);
Point          from_cartesian(const Mesh&, const CartesianPoint&);
Point          random_point(const Mesh&);
PointOnEdge    make_on_edge(const Mesh&, HalfEdgeId, const CartesianPoint&);
PointOnEdge    make_on_edge(const Mesh&, FaceId, const CartesianPoint&);
PointOnEdge    flip(const Mesh&, const PointOnEdge&);
bool           same_location(const Mesh&, const Point&, const Point&);
std::string    describe(const Mesh&, const Point&);

}  // namespace mdv::mesh
#endif  // MDV_MESH_POINT_HPP
