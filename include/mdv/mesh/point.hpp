#ifndef MDV_MESH_POINT_HPP
#define MDV_MESH_POINT_HPP

#include <optional>
#include <variant>

#include "mdv/eigen_defines.hpp"
#include "mdv/macros.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/half_edge.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"

namespace mdv::mesh {

class Point {
public:
    class PointOnVertexDescriptor {
    public:
        PointOnVertexDescriptor(Vertex v, std::optional<Face> f = std::nullopt) :
                _v(std::move(v)), _f(std::move(f)) {}

        MDV_NODISCARD CartesianPoint
        position() const noexcept {
            return _v.position();
        }

        MDV_NODISCARD std::string describe() const;

        MDV_NODISCARD Face face() const;

        MDV_NODISCARD const Mesh&
        mesh() const {
            return _v.mesh();
        }

        MDV_NODISCARD const Vertex&
        vertex() const {
            return _v;
        }

        void assign_face(const Face& f);

    private:
        Vertex              _v;
        std::optional<Face> _f = std::nullopt;
    };

    class PointOnEdgeDescriptor {
    public:
        PointOnEdgeDescriptor(HalfEdge he, double distance) :
                _he(std::move(he)), _c(distance) {
            assert(_c > 0.0 && _c < 1.0);
        }

        PointOnEdgeDescriptor(HalfEdge he, const CartesianPoint& position);

        static PointOnEdgeDescriptor random(const Mesh& mesh);

        MDV_NODISCARD CartesianPoint position() const noexcept;

        MDV_NODISCARD std::string describe() const;

        MDV_NODISCARD Face face() const;

        MDV_NODISCARD const Mesh&
        mesh() const {
            return _he.mesh();
        }

        MDV_NODISCARD PointOnEdgeDescriptor display_in_opposite_halfedge() const;

        MDV_NODISCARD double
        coordinate() const {
            return _c;
        }

        MDV_NODISCARD const HalfEdge&
        halfedge() const {
            return _he;
        }

    private:
        friend class internal::CgalImpl;

        HalfEdge _he;
        double   _c;  // (0, 1) ranged coordinate to describe point location
    };

    class PointInFaceDescriptor {
    public:
        PointInFaceDescriptor(Face face, Vec3d barycentric_coords);

        static PointInFaceDescriptor from_cartesian(
                const Face& f, const Vec3d& position
        );

        MDV_NODISCARD CartesianPoint position() const noexcept;

        MDV_NODISCARD std::string describe() const;

        MDV_NODISCARD Face
        face() const {
            return _f;
        }

        MDV_NODISCARD const Mesh&
        mesh() const {
            return _f.mesh();
        }

        MDV_NODISCARD Vec3d
        coords() const noexcept {
            return _b;
        }

        MDV_NODISCARD bool
        is_undefined() const noexcept {
            return !_f.is_valid();
        }

    private:
        friend class internal::CgalImpl;

        Face  _f;
        Vec3d _b;  //< Barycentric coordinates
    };

    using PointDescriptor = std::variant<
            PointOnVertexDescriptor,
            PointOnEdgeDescriptor,
            PointInFaceDescriptor>;

    Point();

    Point(Vertex v) : _pt(PointOnVertexDescriptor(std::move(v))) {}

    /**
     * @brief Retrieves the closes point on the mesh to the given point described in
     * 3D space.
     *
     */
    static Point from_cartesian(const Mesh& m, const CartesianPoint& pt);

    /**
     * @brief Defines an "undefined" point, i.e. a point with no meaning.
     *
     * Useful to initialise a point into a known state that represent an invalid
     * location.
     *
     */
    static Point undefined(const Mesh& m) noexcept;

    static Point random(const Mesh& m) noexcept;

    MDV_NODISCARD bool is_undefined() const noexcept;

    /**
     * @brief Retrieves the cartesian position of the point.
     *
     */
    MDV_NODISCARD CartesianPoint
    position() const noexcept {
        assert(!is_undefined());
        return std::visit([](const auto& impl) { return impl.position(); }, _pt);
    }

    MDV_NODISCARD
    std::string
    describe() const {
        return std::visit([](const auto& impl) { return impl.describe(); }, _pt);
    };

    MDV_NODISCARD Face
    face() const noexcept {
        assert(!is_undefined());
        return std::visit([](const auto& impl) { return impl.face(); }, _pt);
    };

    MDV_NODISCARD
    const Mesh&
    mesh() const {
        assert(!is_undefined());
        return std::visit(
                [](const auto& impl) -> const Mesh& { return impl.mesh(); }, _pt
        );
    }

    MDV_NODISCARD bool operator==(const Point& other) const noexcept;
    MDV_NODISCARD bool operator!=(const Point& other) const noexcept;

    Point(const Point& other)            = default;
    Point(Point&& other)                 = default;
    Point& operator=(const Point& other) = default;
    Point& operator=(Point&& other)      = default;

    MDV_NODISCARD
    const PointDescriptor&
    descriptor() const {
        return _pt;
    }

    // Yields nullptr if retrieving wrong invariant
    template <typename T>
    const T*
    get_as() const {
        return std::get_if<T>(&_pt);
    }

    Point(const PointOnVertexDescriptor& _pt_impl) : _pt(_pt_impl) {};
    Point(const PointOnEdgeDescriptor& _pt_impl) : _pt(_pt_impl) {};
    Point(const PointInFaceDescriptor& _pt_impl) : _pt(_pt_impl) {};

    Point(PointOnVertexDescriptor&& _pt_impl) : _pt(std::move(_pt_impl)) {};
    Point(PointOnEdgeDescriptor&& _pt_impl) : _pt(std::move(_pt_impl)) {};
    Point(PointInFaceDescriptor&& _pt_impl) : _pt(std::move(_pt_impl)) {};

private:
    PointDescriptor _pt;

    static PointInFaceDescriptor undefined_point;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_POINT_HPP
