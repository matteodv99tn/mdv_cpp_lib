#ifndef MDV_MESH_VERTEX_HPP
#define MDV_MESH_VERTEX_HPP

#include <Eigen/Dense>
#include <gsl/pointers>
#include <iterator>

#include <mdv/macros.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh_element.hpp>

namespace mdv::mesh {

class Vertex : MeshElement {
public:
    /**
     * @brief Instantiates a vertex from a mesh and an index.
     *
     * @note Beware that this constructor can rely on the implicit conversion of Mesh*
     * and const Mesh* to MeshElement.
     */
    Vertex(const MeshElement& mesh, VertexIndex_t index) noexcept;

    /**
     * @brief Retrieves the cartesian position of the vertex
     */
    MDV_NODISCARD Eigen::Vector3d position() const noexcept;


    MDV_NODISCARD VertexIndex_t id() const noexcept;

protected:
    void ensure_not_constant_mesh() const;

private:
    bool operator==(const Vertex&) const noexcept;


    friend class VertexIterator;
    friend class ConstVertexIterator;

    VertexIndex_t _id{0};
};

class VertexIterator {
public:
    // NOLINTBEGIN  naming conventions
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = Vertex;
    using pointer           = Vertex*;
    using reference         = Vertex&;
    // NOLINTEND

    VertexIterator(Mesh* mesh, VertexIndex_t index) noexcept;

    Vertex&         operator*() noexcept;
    VertexIterator& operator++() noexcept;
    VertexIterator  operator++(int) noexcept;
    VertexIterator& operator--() noexcept;
    VertexIterator  operator--(int) noexcept;

    bool operator==(const VertexIterator&) const noexcept;
    bool operator!=(const VertexIterator&) const noexcept;

private:
    Vertex _v;
};

class ConstVertexIterator {
public:
    // NOLINTBEGIN  naming conventions
    using iterator_category = std::bidirectional_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = const Vertex;
    using pointer           = const Vertex*;
    using reference         = const Vertex&;
    // NOLINTEND

    ConstVertexIterator(const Mesh* mesh, VertexIndex_t index) noexcept;

    const Vertex&        operator*() const noexcept;
    ConstVertexIterator& operator++() noexcept;
    ConstVertexIterator  operator++(int) noexcept;
    ConstVertexIterator& operator--() noexcept;
    ConstVertexIterator  operator--(int) noexcept;

    bool operator==(const ConstVertexIterator&) const noexcept;
    bool operator!=(const ConstVertexIterator&) const noexcept;

private:
    Vertex _v;
};


}  // namespace mdv::mesh


#endif  // MDV_MESH_VERTEX_HPP
