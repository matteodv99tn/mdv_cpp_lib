#ifndef MDV_MESH_CLASS_HPP
#define MDV_MESH_CLASS_HPP

#include <Eigen/Geometry>
#include <filesystem>
#include <gsl/pointers>
#include <string_view>

#include <boost/range/iterator_range.hpp>
#include <boost/range/iterator_range_core.hpp>

#include "mdv/macros.hpp"
#include "mdv/mesh/face.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/half_edge.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/logging.hpp"

namespace mdv::mesh {

/**
 * @brief Discrete 2-manifold surface mesh with geodesic and differential tools.
 *
 * Mesh models a discrete 2-manifold surface via a half-edge data structure and
 * provides the core geometric operators used for learning policies on surfaces:
 * geodesic construction, local surface queries, and mesh-level utilities.
 *
 * The class hides CGAL behind a PIMPL (`internal::CgalImpl`) to keep compile time
 * low while exposing a stable, lightweight interface. The CGAL backend is used
 * for loading, AABB queries, and shortest-path computations.
 */
class Mesh {
public:
    /**
     * @brief Default logger used by mesh instances.
     */
    static Logger::SharedPtr default_logger;

    using CgalImpl = internal::CgalImpl;

    // Factory functions
    /**
     * @brief Creates a mesh from a file on disk.
     *
     * @param file_path Path to a mesh file supported by CGAL.
     * @return Loaded mesh.
     */
    static Mesh from_file(const std::filesystem::path& file_path);

    /**
     * @brief Extracts a connected submesh by bounding normal deviation.
     *
     * Starting from a seed point, this function propagates across adjacent faces and
     * keeps faces whose normals stay within the specified angle of the seed face
     * normal. This is useful to isolate locally smooth patches for learning or
     * analysis on surfaces.
     *
     * @param mesh Source mesh.
     * @param pt Seed point on the mesh.
     * @param max_normal_angle Maximum allowed normal deviation in degrees.
     * @return Extracted submesh.
     */
    static Mesh extract_normal_bounded_surface(
            const Mesh& mesh, const Point& pt, double max_normal_angle = 90.0
    );

    // Copy and move constructors/assignment operators are deleted
    /**
     * @brief Mesh is non-copyable.
     */
    Mesh(const Mesh& other) = delete;
    /**
     * @brief Move constructor.
     *
     * @param other Mesh to move from.
     */
    Mesh(Mesh&& other);
    /**
     * @brief Mesh is non-copyable.
     */
    Mesh& operator=(const Mesh& other) = delete;
    /**
     * @brief Move assignment.
     *
     * @param other Mesh to move from.
     * @return Reference to this mesh.
     */
    Mesh& operator=(Mesh&& other) noexcept;

    /**
     * @brief Destructor.
     */
    ~Mesh();

    /**
     * @brief Applies an affine transformation to the mesh geometry.
     *
     * @param transformation Affine transform in 3D.
     */
    void transform(const Eigen::Affine3d& transformation);

    /**
     * @brief Builds a shortest-path geodesic between two surface points.
     *
     * @param from Start point on the mesh.
     * @param to End point on the mesh.
     * @return Geodesic polyline representing the shortest path on the surface.
     */
    MDV_NODISCARD Geodesic build_geodesic(const Point& from, const Point& to) const;

    /**
     * @brief Computes per-vertex normals.
     *
     * @return Vector of vertex normals indexed by vertex id.
     */
    MDV_NODISCARD std::vector<Eigen::Vector3d> compute_vertex_normals() const noexcept;

    // Getters
    /**
     * @brief Number of vertices.
     *
     * @return Number of vertices.
     */
    MDV_NODISCARD std::size_t num_vertices() const;

    /**
     * @brief Number of faces.
     *
     * @return Number of faces.
     */
    MDV_NODISCARD std::size_t num_faces() const;

    /**
     * @brief Mesh name (typically derived from filename).
     *
     * @return Mesh name.
     */
    MDV_NODISCARD std::string_view
                  name() const {
        return _name;
    };

    /**
     * @brief Retrieves a face by index.
     *
     * @param id Face index.
     * @return Face handle.
     */
    MDV_NODISCARD Face
    face(const Index id) const {
        return {*this, id};
    }

    /**
     * @brief Retrieves a vertex by index.
     *
     * @param id Vertex index.
     * @return Vertex handle.
     */
    MDV_NODISCARD Vertex
    vertex(const Index& id) const {
        return Vertex(*this, id);
    }

    /**
     * @brief Logger associated with this mesh instance.
     *
     * @return Logger reference.
     */
    MDV_NODISCARD Logger&
    logger() const {
        assert(_logger != nullptr);
        return *_logger.get();
    };

    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //

#if 0
    // clang-format off
    /**
     * @brief Retrieves an iterator to the beginning of the face range.
     *
     * @return Face::Iterator An iterator to the first face.
     */
    MDV_NODISCARD Face::Iterator      faces_begin() noexcept       { return _faces.begin(); }

    /**
     * @brief Retrieves a const iterator to the beginning of the face range.
     *
     * @return Face::ConstIterator A const iterator to the first face.
     */
    MDV_NODISCARD Face::ConstIterator faces_cbegin() noexcept      { return _faces.cbegin(); }

    /**
     * @brief Retrieves a const iterator to the beginning of the face range (const version).
     *
     * @return Face::ConstIterator A const iterator to the first face.
     */
    MDV_NODISCARD Face::ConstIterator faces_begin() const noexcept { return _faces.begin(); }

    /**
     * @brief Retrieves an iterator to the end of the face range.
     *
     * @return Face::Iterator An iterator past the last face.
     */
    MDV_NODISCARD Face::Iterator      faces_end() noexcept         { return _faces.end(); }

    /**
     * @brief Retrieves a const iterator to the end of the face range.
     *
     * @return Face::ConstIterator A const iterator past the last face.
     */
    MDV_NODISCARD Face::ConstIterator faces_cend() noexcept        { return _faces.cend(); }

    /**
     * @brief Retrieves a const iterator to the end of the face range (const version).
     *
     * @return Face::ConstIterator A const iterator past the last face.
     */
    MDV_NODISCARD Face::ConstIterator faces_end() const noexcept   { return _faces.end(); }

    /**
     * @brief Retrieves a range of all faces in the mesh.
     *
     * @return boost::iterator_range<Face::Iterator> A range of all faces.
     */
    MDV_NODISCARD boost::iterator_range<Face::Iterator> faces() noexcept            { return {faces_begin(), faces_end()}; }

    /**
     * @brief Retrieves a const range of all faces in the mesh (const version).
     *
     * @return boost::iterator_range<Face::ConstIterator> A const range of all faces.
     */
    MDV_NODISCARD boost::iterator_range<Face::ConstIterator> faces() const noexcept { return {faces_begin(), faces_end()}; }

    // clang-format on

    // clang-format off
    /**
     * @brief Retrieves an iterator to the beginning of the vertex range.
     *
     * @return Vertex::Iterator An iterator to the first vertex.
     */
    MDV_NODISCARD Vertex::Iterator      vertices_begin() noexcept       { return _vertices.begin(); }

    /**
     * @brief Retrieves a const iterator to the beginning of the vertex range.
     *
     * @return Vertex::ConstIterator A const iterator to the first vertex.
     */
    MDV_NODISCARD Vertex::ConstIterator vertices_cbegin() noexcept      { return _vertices.cbegin(); }

    /**
     * @brief Retrieves a const iterator to the beginning of the vertex range (const version).
     *
     * @return Vertex::ConstIterator A const iterator to the first vertex.
     */
    MDV_NODISCARD Vertex::ConstIterator vertices_begin() const noexcept { return _vertices.begin(); }

    /**
     * @brief Retrieves an iterator to the end of the vertex range.
     *
     * @return Vertex::Iterator An iterator past the last vertex.
     */
    MDV_NODISCARD Vertex::Iterator      vertices_end() noexcept         { return _vertices.end(); }

    /**
     * @brief Retrieves a const iterator to the end of the vertex range.
     *
     * @return Vertex::ConstIterator A const iterator past the last vertex.
     */
    MDV_NODISCARD Vertex::ConstIterator vertices_cend() noexcept        { return _vertices.cend(); }

    /**
     * @brief Retrieves a const iterator to the end of the vertex range (const version).
     *
     * @return Vertex::ConstIterator A const iterator past the last vertex.
     */
    MDV_NODISCARD Vertex::ConstIterator vertices_end() const noexcept   { return _vertices.end(); }

    /**
     * @brief Retrieves a range of all vertices in the mesh.
     *
     * @return boost::iterator_range<Vertex::Iterator> A range of all vertices.
     */
    MDV_NODISCARD boost::iterator_range<Vertex::Iterator> vertices() noexcept            { return {vertices_begin(), vertices_end()}; }

    /**
     * @brief Retrieves a const range of all vertices in the mesh (const version).
     *
     * @return boost::iterator_range<Vertex::ConstIterator> A const range of all vertices.
     */
    MDV_NODISCARD boost::iterator_range<Vertex::ConstIterator> vertices() const noexcept { return {vertices_begin(), vertices_end()}; }

    // clang-format on
#endif

    /**
     * @brief Access CGAL implementation (mutable).
     *
     * @return Reference to CGAL backend.
     */
    MDV_NODISCARD CgalImpl&
    cgal() {
        assert(_impl);
        return *_impl;
    }

    /**
     * @brief Access CGAL implementation (const).
     *
     * @return Const reference to CGAL backend.
     */
    MDV_NODISCARD const CgalImpl&
    cgal() const {
        assert(_impl);
        return *_impl;
    }

    /**
     * @brief Retrieves a random face from the mesh.
     *
     * @return Random face.
     */
    MDV_NODISCARD Face random_face() const;

    /**
     * @brief Constructs an Nx3 matrix with all vertex positions.
     *
     * @return Vertex matrix (N x 3).
     */
    MDV_NODISCARD Eigen::MatrixXd get_vertex_matrix() const;

    /**
     * @brief Constructs an Nx3 matrix of face vertex indices.
     *
     * @return Face index matrix (N x 3).
     */
    MDV_NODISCARD Eigen::MatrixXi get_face_matrix() const;

    /**
     * @brief Constructs an Nx3 matrix of face indices as doubles.
     *
     * This is intended for Python bindings where integer matrices are less
     * convenient. Use `get_face_matrix()` for native C++ workflows.
     *
     * @return Face index matrix (N x 3) as doubles.
     */
    MDV_NODISCARD Eigen::MatrixXd get_face_matrix_double() const;

    /**
     * @brief Returns the closest vertex to a 3D point.
     *
     * @param pt Query point in 3D.
     * @return Closest vertex on the mesh.
     */
    MDV_NODISCARD Vertex closest_vertex(const CartesianPoint& pt);

private:
    /**
     * @brief Constructs a mesh from a CGAL backend and name.
     *
     * @param data CGAL backend pointer.
     * @param name Mesh name.
     */
    Mesh(gsl::owner<CgalImpl*> data, const std::string& name);

    /**
     * @brief Logger associated with the mesh instance.
     */
    Logger::SharedPtr _logger = default_logger;

    /**
     * @brief CGAL backend (PIMPL) owning the heavy implementation data.
     */
    gsl::owner<CgalImpl*> _impl = nullptr;

    /**
     * @brief Mesh name for logging and debugging.
     */
    std::string _name;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPP
