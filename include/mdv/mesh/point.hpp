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

/**
 * @brief Point on a discrete surface mesh.
 *
 * A point is represented by one of three descriptors: on-vertex, on-edge, or
 * in-face. This abstraction is central to geodesic computations and to mapping
 * trajectories and policies onto a surface, as well as to preserve numerical and
 * semantic accuracy.
 */
class Point {
public:
    /**
     * @brief Descriptor for a point located on a mesh vertex.
     */
    class PointOnVertexDescriptor {
    public:
        /**
         * @brief Creates a vertex descriptor, optionally binding a face.
         *
         * @param v Vertex hosting the point.
         * @param f Optional incident face.
         */
        PointOnVertexDescriptor(Vertex v, std::optional<Face> f = std::nullopt) :
                _v(std::move(v)), _f(std::move(f)) {}

        /**
         * @brief Cartesian position of the vertex.
         *
         * @return Vertex position.
         */
        MDV_NODISCARD CartesianPoint
        position() const noexcept {
            return _v.position();
        }

        /**
         * @brief Human-readable description.
         *
         * @return Description string.
         */
        MDV_NODISCARD std::string describe() const;

        /**
         * @brief Returns a face incident to the vertex.
         *
         * @return Incident face.
         */
        MDV_NODISCARD Face face() const;

        /**
         * @brief Owning mesh.
         *
         * @return Mesh reference.
         */
        MDV_NODISCARD const Mesh&
        mesh() const {
            return _v.mesh();
        }

        /**
         * @brief Underlying vertex.
         *
         * @return Vertex reference.
         */
        MDV_NODISCARD const Vertex&
        vertex() const {
            return _v;
        }

        /**
         * @brief Binds this descriptor to a specific incident face.
         *
         * Throws if the face to bind does not contain the vertex.
         *
         * @param f Face to bind.
         */
        void assign_face(const Face& f);

    private:
        Vertex              _v;
        std::optional<Face> _f = std::nullopt;
    };

    /**
     * @brief Descriptor for a point on an edge (half-edge + coordinate).
     */
    class PointOnEdgeDescriptor {
    public:
        /**
         * @brief Creates an edge descriptor with a normalized coordinate.
         *
         * @param he Half-edge hosting the point.
         * @param distance Normalized coordinate in (0, 1).
         */
        PointOnEdgeDescriptor(HalfEdge he, double distance) :
                _he(std::move(he)), _c(distance) {
            assert(_c > 0.0 && _c < 1.0);
        }

        /**
         * @brief Creates an edge descriptor from a 3D position on the edge.
         *
         * @param he Half-edge hosting the point.
         * @param position Cartesian position on the edge.
         */
        PointOnEdgeDescriptor(HalfEdge he, const CartesianPoint& position);

        /**
         * @brief Creates an edge descriptor from a 3D position on the edge from a 
         * specific face.
         *
         * @param f Face whose edge contains the given position
         * @param position Cartesian position on the edge.
         */
        static PointOnEdgeDescriptor from_face(const Face& f, const CartesianPoint& position);

        /**
         * @brief Random edge descriptor on a mesh.
         *
         * @param mesh Source mesh.
         * @return Random edge descriptor.
         */
        static PointOnEdgeDescriptor random(const Mesh& mesh);

        /**
         * @brief Cartesian position on the edge.
         *
         * @return Cartesian position.
         */
        MDV_NODISCARD CartesianPoint position() const noexcept;

        /**
         * @brief Human-readable description.
         *
         * @return Description string.
         */
        MDV_NODISCARD std::string describe() const;

        /**
         * @brief Face incident to this half-edge.
         *
         * @return Incident face.
         */
        MDV_NODISCARD Face face() const;

        /**
         * @brief Owning mesh.
         *
         * @return Mesh reference.
         */
        MDV_NODISCARD const Mesh&
        mesh() const {
            return _he.mesh();
        }

        /**
         * @brief Returns the same point expressed on the opposite half-edge.
         *
         * @return Equivalent descriptor on the opposite half-edge.
         */
        MDV_NODISCARD PointOnEdgeDescriptor display_in_opposite_halfedge() const;

        /**
         * @brief Normalized coordinate in (0, 1) along the half-edge.
         *
         * @return Coordinate along the edge.
         */
        MDV_NODISCARD double
        coordinate() const {
            return _c;
        }

        /**
         * @brief Underlying half-edge.
         *
         * @return Half-edge reference.
         */
        MDV_NODISCARD const HalfEdge&
        halfedge() const {
            return _he;
        }

    private:
        friend class internal::CgalImpl;

        HalfEdge _he;
        double   _c;  // (0, 1) ranged coordinate to describe point location
    };

    /**
     * @brief Descriptor for a point inside a face via barycentric coordinates.
     */
    class PointInFaceDescriptor {
    public:
        /**
         * @brief Creates a face descriptor from barycentric coordinates.
         *
         * @param face Face containing the point.
         * @param barycentric_coords Barycentric coordinates.
         */
        PointInFaceDescriptor(Face face, Vec3d barycentric_coords);

        /**
         * @brief Computes barycentric coordinates from a 3D position on a face.
         *
         * @param f Face containing the point.
         * @param position Cartesian position on the face.
         * @return Face descriptor with barycentric coordinates.
         */
        static PointInFaceDescriptor from_cartesian(
                const Face& f, const Vec3d& position
        );

        /**
         * @brief Cartesian position of the barycentric point.
         *
         * @return Cartesian position.
         */
        MDV_NODISCARD CartesianPoint position() const noexcept;

        /**
         * @brief Human-readable description.
         *
         * @return Description string.
         */
        MDV_NODISCARD std::string describe() const;

        /**
         * @brief Face containing the point.
         *
         * @return Face containing the point.
         */
        MDV_NODISCARD Face
        face() const {
            return _f;
        }

        /**
         * @brief Owning mesh.
         *
         * @return Mesh reference.
         */
        MDV_NODISCARD const Mesh&
        mesh() const {
            return _f.mesh();
        }

        /**
         * @brief Barycentric coordinates (sum to 1).
         *
         * @return Barycentric coordinate vector.
         */
        MDV_NODISCARD Vec3d
        coords() const noexcept {
            return _b;
        }

        /**
         * @brief True if this descriptor represents an undefined face.
         *
         * @return True if undefined.
         */
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

    /**
     * @brief Constructs an undefined point.
     */
    Point();

    /**
     * @brief Constructs a point on a vertex.
     *
     * @param v Vertex hosting the point.
     */
    Point(Vertex v) : _pt(PointOnVertexDescriptor(std::move(v))) {}

    /**
     * @brief Retrieves the closest point on the mesh to a 3D position.
     *
     * @param m Mesh on which to project.
     * @param pt Query point in 3D.
     * @return Closest point on the mesh.
     */
    static Point from_cartesian(const Mesh& m, const CartesianPoint& pt);

    /**
     * @brief Returns an undefined point bound to a mesh.
     *
     * @param m Mesh to bind the undefined point to.
     * @return Undefined point.
     */
    static Point undefined(const Mesh& m) noexcept;

    /**
     * @brief Random point inside a random face.
     *
     * @param m Mesh to sample from.
     * @return Random point on the mesh.
     */
    static Point random(const Mesh& m) noexcept;

    /**
     * @brief True if this point is undefined.
     *
     * @return True if undefined.
     */
    MDV_NODISCARD bool is_undefined() const noexcept;

    /**
     * @brief Cartesian position on the surface.
     *
     * @return Cartesian position.
     */
    MDV_NODISCARD CartesianPoint
    position() const noexcept {
        assert(!is_undefined());
        return std::visit([](const auto& impl) { return impl.position(); }, _pt);
    }

    /**
     * @brief Human-readable description.
     *
     * @return Description string.
     */
    MDV_NODISCARD
    std::string
    describe() const {
        return std::visit([](const auto& impl) { return impl.describe(); }, _pt);
    };

    /**
     * @brief Face containing the point.
     *
     * @return Face containing the point.
     */
    MDV_NODISCARD Face
    face() const noexcept {
        assert(!is_undefined());
        return std::visit([](const auto& impl) { return impl.face(); }, _pt);
    };

    /**
     * @brief Owning mesh.
     *
     * @return Mesh reference.
     */
    MDV_NODISCARD
    const Mesh&
    mesh() const {
        assert(!is_undefined());
        return std::visit(
                [](const auto& impl) -> const Mesh& { return impl.mesh(); }, _pt
        );
    }

    /**
     * @brief Equality by 3D position.
     *
     * @param other Point to compare.
     * @return True if positions are equal.
     */
    MDV_NODISCARD bool operator==(const Point& other) const noexcept;

    /**
     * @brief Inequality by 3D position.
     *
     * @param other Point to compare.
     * @return True if positions differ.
     */
    MDV_NODISCARD bool operator!=(const Point& other) const noexcept;

    Point(const Point& other)            = default;
    Point(Point&& other)                 = default;
    Point& operator=(const Point& other) = default;
    Point& operator=(Point&& other)      = default;

    /**
     * @brief Returns the underlying point descriptor.
     *
     * @return Variant descriptor.
     */
    MDV_NODISCARD
    const PointDescriptor&
    descriptor() const {
        return _pt;
    }

    /**
     * @brief Returns a pointer to the requested descriptor type.
     *
     * Returns null if variant is not containing requested description.
     *
     * @tparam T Descriptor type.
     * @return Pointer to descriptor or nullptr.
     */
    template <typename T>
    const T*
    get_as() const {
        return std::get_if<T>(&_pt);
    }

    /**
     * @brief Constructs from a vertex descriptor.
     *
     * @param _pt_impl Vertex descriptor.
     */
    Point(const PointOnVertexDescriptor& _pt_impl) : _pt(_pt_impl) {};

    /**
     * @brief Constructs from an edge descriptor.
     *
     * @param _pt_impl Edge descriptor.
     */
    Point(const PointOnEdgeDescriptor& _pt_impl) : _pt(_pt_impl) {};

    /**
     * @brief Constructs from a face descriptor.
     *
     * @param _pt_impl Face descriptor.
     */
    Point(const PointInFaceDescriptor& _pt_impl) : _pt(_pt_impl) {};

    /**
     * @brief Constructs from a vertex descriptor (move).
     *
     * @param _pt_impl Vertex descriptor.
     */
    Point(PointOnVertexDescriptor&& _pt_impl) : _pt(std::move(_pt_impl)) {};

    /**
     * @brief Constructs from an edge descriptor (move).
     *
     * @param _pt_impl Edge descriptor.
     */

    Point(PointOnEdgeDescriptor&& _pt_impl) : _pt(std::move(_pt_impl)) {};

    /**
     * @brief Constructs from a face descriptor (move).
     *
     * @param _pt_impl Face descriptor.
     */
    Point(PointInFaceDescriptor&& _pt_impl) : _pt(std::move(_pt_impl)) {};

private:
    PointDescriptor _pt;

    /**
     * @brief Static undefined face descriptor for invalid points.
     */
    static PointInFaceDescriptor undefined_point;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_POINT_HPP
