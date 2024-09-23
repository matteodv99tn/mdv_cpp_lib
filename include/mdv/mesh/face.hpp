#ifndef MDV_MESH_FACE_HPP
#define MDV_MESH_FACE_HPP

#include <array>
#include <Eigen/Dense>
#include <gsl/pointers>
#include <iterator>

#include <mdv/eigen_defines.hpp>
#include <mdv/macros.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh_element.hpp>

namespace mdv::mesh {

class Face : public MeshElement {
public:
    /**
     * @brief Instantiates a vertex from a mesh and an index.
     *
     * @note Beware that this constructor can rely on the implicit conversion of Mesh*
     * and const Mesh* to MeshElement.
     */
    Face(const MeshElement& mesh, FaceIndex_t index) noexcept;

    MDV_NODISCARD FaceIndex_t id() const;

    /**
     * @brief Normal vector to the triangle
     *
     */
    MDV_NODISCARD Vec3_t normal() const;

    /**
     * @brief Yields the vertex of the face using face indexing, zero-based.
     *
     * @param[in] i  index of the vertex. Checkd to be in the range [0, 2].
     */
    MDV_NODISCARD Vertex vertex(long i) const;

    /**
     * @brief Yields the set of vertices that make up the face.
     */
    MDV_NODISCARD std::array<Vertex, 3> vertices() const;

private:
    /**
     * @brief Yields the ordered triplet of indices of the vertex indexes that make up
     * the face.
     */
    MDV_NODISCARD std::array<VertexIndex_t, 3> vertices_ids() const;

    /**
     * @brief Finds the neighbouring faces of the current face.
     *
     * @note The function returns id = -1 if an edge has no neigbour.
     */
    MDV_NODISCARD std::array<FaceIndex_t, 3> neighbour_faces_ids() const;

    bool operator==(const Face&) const noexcept;


    friend class Mesh;
    friend class FaceIterator;
    friend class ConstFaceIterator;

    FaceIndex_t _id{0};
};

class FaceIterator {
public:
    // NOLINTBEGIN  naming conventions
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = Face;
    using pointer           = Face*;
    using reference         = Face&;
    // NOLINTEND

    FaceIterator(Mesh* mesh, FaceIndex_t index) noexcept;

    Face&         operator*() noexcept;
    FaceIterator& operator++() noexcept;
    FaceIterator  operator++(int) noexcept;
    FaceIterator& operator--() noexcept;
    FaceIterator  operator--(int) noexcept;

    bool operator==(const FaceIterator&) const noexcept;
    bool operator!=(const FaceIterator&) const noexcept;

private:
    Face _v;
};

class ConstFaceIterator {
public:
    // NOLINTBEGIN  naming conventions
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = const Face;
    using pointer           = const Face*;
    using reference         = const Face&;
    // NOLINTEND

    ConstFaceIterator(const Mesh* mesh, FaceIndex_t index) noexcept;

    const Face&        operator*() const noexcept;
    ConstFaceIterator& operator++() noexcept;
    ConstFaceIterator  operator++(int) noexcept;
    ConstFaceIterator& operator--() noexcept;
    ConstFaceIterator  operator--(int) noexcept;

    bool operator==(const ConstFaceIterator&) const noexcept;
    bool operator!=(const ConstFaceIterator&) const noexcept;

private:
    Face _v;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_FACE_HPP
