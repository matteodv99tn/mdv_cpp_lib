#ifndef MDV_MESH_SAMPLER_HPP
#define MDV_MESH_SAMPLER_HPP

#include <cstdint>
#include <random>
#include <vector>

#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"

namespace mdv::mesh {

/**
 * @brief Seedable random sampler of mesh elements.
 *
 * Holds a non-owning reference: the mesh must outlive the sampler.
 * @throws std::runtime_error if the mesh is empty.
 */
class Sampler {
public:
    Sampler(const Mesh& mesh, std::uint64_t seed);
    MDV_NODISCARD FaceId        face();
    MDV_NODISCARD PointOnVertex point_on_vertex();
    MDV_NODISCARD PointOnEdge   point_on_edge();
    MDV_NODISCARD PointInFace   point_in_face();
    /** @brief Sample a unit tangent at a point strictly inside a face. */
    MDV_NODISCARD TangentVector unit_tangent(const Point& application_point);
    MDV_NODISCARD std::vector<Point> points_on_vertices(std::size_t n);
    MDV_NODISCARD std::vector<Point> points_on_edges(std::size_t n);
    MDV_NODISCARD std::vector<Point> points_in_faces(std::size_t n);

private:
    const Mesh*     _mesh;
    std::mt19937_64 _gen;
};

}  // namespace mdv::mesh
#endif  // MDV_MESH_SAMPLER_HPP
