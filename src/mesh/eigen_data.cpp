#include "mdv/mesh/eigen_data.hpp"

#include <spdlog/spdlog.h>
#include <thread>

#include <range/v3/algorithm/contains.hpp>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"

namespace rs = ranges;

constexpr mdv::mesh::IndexTriplet invalid_indexes{
        mdv::mesh::invalid_index, mdv::mesh::invalid_index, mdv::mesh::invalid_index};

using mdv::mesh::internal::CgalImpl;
using mdv::mesh::internal::EigenData;

EigenData::EigenData(const CgalImpl& cgal, spdlog::logger& logger) {
    const std::size_t n_faces    = cgal._mesh.num_faces();
    const std::size_t n_vertices = cgal._mesh.num_vertices();

    logger.trace("Syncing vertex matrix");
    vertices.resize(n_vertices);
    for (const CgalImpl::Mesh::Vertex_index& vertex : cgal._mesh.vertices()) {
        const auto cgal_vertex = cgal._mesh.point(vertex);
        vertices[vertex.idx()] =
                Eigen::Vector3d(cgal_vertex.x(), cgal_vertex.y(), cgal_vertex.z());
    }

    logger.trace("Syncing faces matrix");
    faces.resize(n_faces);
    for (const CgalImpl::Mesh::Face_index& face : cgal._mesh.faces()) {
        const auto vertex_iter =
                CGAL::vertices_around_face(cgal._mesh.halfedge(face), cgal._mesh);
        int i = 0;
        for (const auto& vi : vertex_iter) {
            Expects(i < 3);
            faces[static_cast<long>(face)].at(i) = vi.idx();
            i++;
        }
    }

    logger.trace("Finding neighbouring faces");
    // Helper function that checks if two vertices are neighbours of a given face
    auto is_neighbour = [](const IndexTriplet& face,
                           const Index&        v1_id,
                           const Index&        v2_id) -> bool {
        const bool v1_found = rs::contains(face, v1_id);
        const bool v2_found = rs::contains(face, v2_id);
        return v1_found && v2_found;
    };


    // Iterates all faces looking for neighbour faces
    auto find_face_neighbour = [this, is_neighbour](const Index& face_id) -> void {
        auto& f_mat                      = faces;
        auto& neigh_faces                = neighbour_faces;
        const auto [v1_id, v2_id, v3_id] = f_mat[face_id];

        assert(neigh_faces[face_id] == invalid_indexes);
        for (Index i = 0; i < f_mat.size(); ++i) {
            if (i == face_id) continue;

            if (is_neighbour(f_mat[i], v1_id, v2_id)) {
                assert(neigh_faces[face_id][0] == invalid_index);
                neigh_faces[face_id][0] = i;
            }
            if (is_neighbour(f_mat[i], v1_id, v3_id)) {
                assert(neigh_faces[face_id][1] == invalid_index);
                neigh_faces[face_id][1] = i;
            }
            if (is_neighbour(f_mat[i], v2_id, v3_id)) {
                assert(neigh_faces[face_id][2] == invalid_index);
                neigh_faces[face_id][2] = i;
            }
        }
    };

    // Helper function to find neighbours in batches (to enable efficient parallelism)
    auto batch_find_process_neighbours =
            [this, find_face_neighbour](const Index& start, const Index& end) -> void {
        for (auto i = start; i < end; ++i) find_face_neighbour(i);
    };

    // Allocate memory for neighbouring faces and populate them
    auto& neigh_faces = neighbour_faces;
    neigh_faces.resize(n_faces, invalid_indexes);
    const auto                batch_size = 1000;
    std::vector<std::jthread> threads;
    for (Index i = 0; i * batch_size < n_faces; i++) {
        // threads.emplace_back(find_face_neighbour, i);
        const auto start_id = i * batch_size;
        const auto end_id   = (i + 1) * batch_size;
        threads.emplace_back(
                batch_find_process_neighbours,
                start_id,
                std::min(end_id, static_cast<Index>(n_faces))
        );
    }
}
