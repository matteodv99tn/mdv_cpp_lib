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
 * @brief Represents a mesh composed of vertices, faces, and half-edges.
 *
 * This class encapsulates the necessary data and functionality to work with
 * meshes. It provides methods for loading, processing, and querying mesh data.
 *
 * The mesh stores data as half-edge data structure, and internally uses CGAL
 * to load the data and perform some operations.
 */
class Mesh {
public:
    static Logger::SharedPtr default_logger;

    using CgalImpl = internal::CgalImpl;

    // Factory functions
    /**
     * @brief Creates a new mesh from a file.
     *
     * @param file_path The path to the file containing the mesh data.
     * @return Mesh A new mesh object loaded from the specified file.
     */
    static Mesh from_file(const std::filesystem::path& file_path);

    // Copy and move constructors/assignment operators are deleted
    Mesh(const Mesh& other) = delete;
    Mesh(Mesh&& other);
    Mesh& operator=(const Mesh& other) = delete;
    Mesh& operator=(Mesh&& other) noexcept;

    /**
     * @brief Destructor.
     *
     * Cleans up any resources associated with the mesh.
     */
    ~Mesh();

    /**
     * @brief Applies a transformation to the mesh.
     *
     * @param transformation The affine transformation to apply.
     */
    void transform(const Eigen::Affine3d& transformation);

    /**
     * @brief Builds a geodesic path between two points on the mesh.
     *
     * @param from The starting point of the geodesic.
     * @param to The ending point of the geodesic.
     * @return Geodesic The constructed geodesic path.
     */
    MDV_NODISCARD Geodesic build_geodesic(const Point& from, const Point& to) const;

    /**
     * @brief Computes vertex normals for the mesh.
     *
     * @return std::vector<Eigen::Vector3d> A vector of normal vectors for each vertex.
     */
    MDV_NODISCARD std::vector<Eigen::Vector3d> compute_vertex_normals() const noexcept;

    // Getters
    /**
     * @brief Retrieves the number of vertices in the mesh.
     *
     * @return std::size_t The number of vertices.
     */
    MDV_NODISCARD std::size_t num_vertices() const;

    /**
     * @brief Retrieves the number of faces in the mesh.
     *
     * @return std::size_t The number of faces.
     */
    MDV_NODISCARD std::size_t num_faces() const;

    /**
     * @brief Retrieves the name of the mesh.
     *
     * @return std::string_view A view of the mesh's name.
     */
    MDV_NODISCARD std::string_view
                  name() const {
                      return _name;
    };

    // clang-format off
    /**
     * @brief Retrieves a face by its index.
     *
     * @param id The index of the face to retrieve.
     * @return const Face& A reference to the retrieved face.
     */
    MDV_NODISCARD const Face&   face(const Index& id) const   { return _faces[id]; }

    /**
     * @brief Retrieves a vertex by its index.
     *
     * @param id The index of the vertex to retrieve.
     * @return const Vertex& A reference to the retrieved vertex.
     */
    MDV_NODISCARD const Vertex& vertex(const Index& id) const { return _vertices[id]; }

    /**
     * @brief Retrieves the logger associated with the mesh.
     *
     * @return Logger& A reference to the logger.
     */
    MDV_NODISCARD Logger&       logger() const                { assert(_logger != nullptr); return *_logger.get(); };

    // clang-format on

    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //

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

    // clang-format off
    /**
     * @brief Retrieves the CGAL implementation data associated with the mesh.
     *
     * @return CgalImpl& A reference to the CGAL implementation data.
     */
    MDV_NODISCARD CgalImpl&        cgal()       { assert(_impl); return *_impl; }

    /**
     * @brief Retrieves the const CGAL implementation data associated with the mesh.
     *
     * @return const CgalImpl& A const reference to the CGAL implementation data.
     */
    MDV_NODISCARD const CgalImpl&  cgal() const { assert(_impl); return *_impl; }

    // clang-format on

    /**
     * @brief Retrieves a random face from the mesh.
     *
     * @return const Face& A reference to a randomly selected face.
     */
    const Face& random_face() const;

    /**
     * @brief Constructs the Nx3 matrix with all vertices of the mesh
     *
     */
    MDV_NODISCARD Eigen::MatrixXd get_vertex_matrix() const;

    /**
     * @brief Constructs the Nx3 matrix with all faces of the mesh
     *
     */
    MDV_NODISCARD Eigen::MatrixXi get_face_matrix() const;

    /**
     * @brief Constructs the Nx3 matrix with all faces of the mesh
     *
     * This function returns indices as doubles, which is ultimately not correct and
     * error prone, but is necessary for having proper python bindings
     */
    MDV_NODISCARD Eigen::MatrixXd get_face_matrix_double() const;

private:
    Mesh(gsl::owner<CgalImpl*> data, const std::string& name);

    // Members
    /**
     * @brief The logger associated with the mesh.
     *
     * @var Logger::SharedPtr _logger
     */
    Logger::SharedPtr _logger = default_logger;
    /**
     * @brief The CGAL implementation data associated with the mesh.
     *
     * @var gsl::owner<CgalImpl*> _impl
     */
    gsl::owner<CgalImpl*> _impl = nullptr;
    /**
     * @brief The name of the mesh.
     *
     * @var std::string _name
     */
    std::string _name;
    /**
     * @brief A vector of vertices in the mesh.
     *
     * @var Vertex::Vector _vertices
     */
    Vertex::Vector _vertices;
    /**
     * @brief A vector of faces in the mesh.
     *
     * @var Face::Vector _faces
     */
    Face::Vector _faces;
    /**
     * @brief A vector of half-edges in the mesh.
     *
     * @var HalfEdge::Vector _half_edges
     */
    HalfEdge::Vector _half_edges;

    /**
     * @brief Builds the half-edges data structure.
     *
     * Initializes the half-edges based on the vertices and faces of the mesh.
     */
    void build_halfedges();

    /**
     * @brief Adds a face to the mesh.
     *
     * @param v_ids A triplet of vertex indices defining the face.
     */
    void add_face(const IndexTriplet& v_ids);

    /**
     * @brief Constructs opposite half-edges for each edge in the mesh.
     *
     * Ensures that each edge has a corresponding opposite edge.
     */
    void construct_opposite_halfedges();

    /**
     * @brief Fills in additional information for each half-edge.
     *
     * Completes the half-edges data structure with necessary connections and
     * properties.
     */
    void fill_halfedges();

    /**
     * @brief Checks if the mesh's data structure is correctly initialized.
     *
     * Validates that all half-edges, faces, and vertices are properly connected.
     *
     * @return bool True if the data structure is correctly initialized; false
     * otherwise.
     */
    bool datastructure_correctly_initialised() const;

    // clang-format off
    /**
     * @brief Emplaces a new vertex into the mesh.
     *
     * @param pos The position of the vertex.
     * @return Vertex& A reference to the newly emplaced vertex.
     */
    Vertex&   emplace_vertex(const CartesianPoint& pos) { return _vertices.emplace_back(*this, pos); } 

    /**
     * @brief Emplaces a new face into the mesh.
     *
     * @return Face& A reference to the newly emplaced face.
     */
    Face&     emplace_face()                            { return _faces.emplace_back(*this); }

    /**
     * @brief Emplaces a new half-edge into the mesh.
     *
     * @return HalfEdge& A reference to the newly emplaced half-edge.
     */
    HalfEdge& emplace_halfedge()                        { return _half_edges.emplace_back(*this); }

    // clang-format on


    // When moving a mesh object, mesh pointers in vertices/faces must be updated
    // accordingly
    void update_mesh_element_references();
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPP
