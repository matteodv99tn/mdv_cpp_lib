#ifndef MDV_MESH_FWD_HPP
#define MDV_MESH_FWD_HPP

#include <array>
#include <compare>
#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <type_traits>
#include <vector>

#include "mdv/macros.hpp"

/**
 * @brief Forward declarations and core type aliases for mdv::mesh.
 *
 * These aliases represent intrinsic geometry types used throughout surface learning and
 * geodesic computations.
 */
namespace mdv::mesh {

/**
 * @brief Discrete surface mesh.
 */
class Mesh;

/**
 * @brief Mesh vertex.
 */
class Vertex;
/**
 * @brief Mesh face.
 */
class Face;
/**
 * @brief Mesh half-edge.
 */
class HalfEdge;

/**
 * @brief Point on a mesh (vertex/edge/face).
 */
class Point;

/**
 * @brief Tangent space on the mesh (forward declaration).
 */
class TangentSpace;

/**
 * @brief Index type for mesh entities.
 */
using Index = unsigned int;

/**
 * @brief Index triplet for triangle faces.
 */
using IndexTriplet = std::array<Index, 3>;
// Note: preferred std::array over Eigen::Vector to leverage unpacking features

/**
 * @brief Sentinel invalid index.
 */
constexpr Index invalid_index = std::numeric_limits<Index>::max();

namespace detail {

    /**
     * @brief Mesh-free index interpreted against the corresponding mesh data.
     *
     * Moving a Mesh preserves handles. Topology edits, removal and index compaction
     * may invalidate them; supplying the correct mesh is a caller precondition.
     */
    template <typename Tag>
    struct Handle {
        Index value = invalid_index;

        constexpr Handle() = default;

        constexpr explicit Handle(Index v) noexcept : value(v) {}

        MDV_NODISCARD constexpr bool
        is_valid() const noexcept {
            return value != invalid_index;
        }

        MDV_NODISCARD constexpr explicit
        operator Index() const noexcept {
            return value;
        }

        auto operator<=>(const Handle&) const = default;
    };

    struct VertexTag {};

    struct FaceTag {};

    struct HalfEdgeTag {};
}  // namespace detail

using VertexId   = detail::Handle<detail::VertexTag>;
using FaceId     = detail::Handle<detail::FaceTag>;
using HalfEdgeId = detail::Handle<detail::HalfEdgeTag>;

static_assert(std::is_trivially_copyable_v<VertexId>);
static_assert(std::is_trivially_copyable_v<FaceId>);
static_assert(std::is_trivially_copyable_v<HalfEdgeId>);
static_assert(sizeof(VertexId) == 4);
static_assert(sizeof(FaceId) == 4);
static_assert(sizeof(HalfEdgeId) == 4);
static_assert(!std::is_convertible_v<Index, VertexId>);
static_assert(!std::is_convertible_v<Index, FaceId>);
static_assert(!std::is_convertible_v<Index, HalfEdgeId>);
static_assert(!std::is_convertible_v<VertexId, FaceId>);
static_assert(!std::is_convertible_v<VertexId, HalfEdgeId>);
static_assert(!std::is_convertible_v<FaceId, HalfEdgeId>);
static_assert(!std::is_constructible_v<FaceId, VertexId>);
static_assert(!std::is_constructible_v<HalfEdgeId, FaceId>);
static_assert(!std::is_constructible_v<VertexId, HalfEdgeId>);
static_assert(!std::is_convertible_v<VertexId, Index>);
static_assert(!std::is_convertible_v<FaceId, Index>);
static_assert(!std::is_convertible_v<HalfEdgeId, Index>);

/**
 * @brief 3D point in ambient space.
 */
using CartesianPoint = Eigen::Vector3d;

/**
 * @brief Geodesic polyline represented as 3D points.
 */
using Geodesic = std::vector<CartesianPoint>;

// Internals
namespace internal {
    /**
     * @brief CGAL implementation detail (PIMPL).
     */
    class CgalImpl;


}  // namespace internal

}  // namespace mdv::mesh

template <typename Tag>
struct std::hash<mdv::mesh::detail::Handle<Tag>> {
    std::size_t
    operator()(mdv::mesh::detail::Handle<Tag> id) const noexcept {
        return std::hash<mdv::mesh::Index>{}(id.value);
    }
};


#endif  // MDV_MESH_FWD_HPP
