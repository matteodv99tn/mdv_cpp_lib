#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include "mdv/macros.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/index_element.hpp"
#include "mdv/mesh/mesh_iterator.hpp"

namespace mdv::mesh {
class Face : public internal::IndexBasedElement {
public:
    using Index         = long;
    using IndexTriplet  = ::mdv::mesh::IndexTriplet;
    using Iterator      = MeshIterator<Face, Index>;
    using VertexTriplet = std::array<Vertex, 3>;

    Face() = default;
    Face(const internal::MeshData& m, const Index& id) : IndexBasedElement(m, id) {};

    static Face random(const Mesh& m);

    MDV_NODISCARD Face
    neighbour_face(const Index& edge_id) const {
        assert(edge_id >= 0 && edge_id < 3);
        const auto new_face_id = eigen_data().neighbour_faces[id()][edge_id];
        return {data(), new_face_id};
    }

    /**
     * @brief Normal vector to the triangle
     *
     */
    MDV_NODISCARD Eigen::Vector3d normal() const;

    /**
     * @brief Yields the vertex of the face using face indexing, zero-based.
     *
     * @param[in] i  index of the vertex. Checked to be in the range [0, 2].
     */
    MDV_NODISCARD Vertex vertex(long i) const;

    /**
     * @brief Yields the set of vertices that make up the face.
     */
    MDV_NODISCARD VertexTriplet vertices() const;

    /**
     * @brief Returns the (ordered) triplet of indices of the vertex indices.
     *
     */
    MDV_NODISCARD const IndexTriplet&
    vertices_ids() const {
        return eigen_data().faces[_id];
    }

    bool
    operator==(const Face& other) const noexcept {
        return (_mesh_data == other._mesh_data) && (_id == other._id);
    }


private:
    friend class MeshIterator<Face, Index>;
    friend class Point;

    // clang-format off
        void set_id(const Index& id) noexcept { _id = id; }

        /**
         * @brief Return the position of the first vertex within the face.
         *
         */
        MDV_NODISCARD const CartesianPoint& v1() const noexcept { return eigen_data().vertices[eigen_data().faces[_id][0]]; };

        /**
         * @brief Return the position of the second vertex within the face.
         *
         */
        MDV_NODISCARD const CartesianPoint& v2() const noexcept { return eigen_data().vertices[eigen_data().faces[_id][1]]; };

        /**
         * @brief Return the position of the third vertex within the face.
         *
         */
        MDV_NODISCARD const CartesianPoint& v3() const noexcept { return eigen_data().vertices[eigen_data().faces[_id][2]]; };

        MDV_NODISCARD UvMap compute_uv_map() const;// { assert(_id != -1); return {v1(), v2(), v3()}; };

    // clang-format on
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_FACE_HPP
