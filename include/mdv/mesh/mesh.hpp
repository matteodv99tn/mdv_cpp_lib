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

class Mesh {
public:
    static Logger::SharedPtr default_logger;

    // Forward declarations
    using CgalImpl = internal::CgalImpl;

    // enum GeodesicBuilderPolicy : std::uint8_t {
    //     InternalState,
    //     ExecutionLocal
    // };

    // Factory functions
    static Mesh from_file(const std::filesystem::path& file_path);

    Mesh(const Mesh& other) = delete;
    Mesh(Mesh&& other);
    Mesh& operator=(const Mesh& other) = delete;
    Mesh& operator=(Mesh&& other)      = delete;
    ~Mesh();

    void transform(const Eigen::Affine3d& transformation);

    /**
     * @brief Builds a geodesic path between two points.
     *
     * @param from The starting point of the geodesic.
     * @param to The ending point of the geodesic.
     *
     * @return Geodesic The constructed geodesic path.
     */
    MDV_NODISCARD Geodesic build_geodesic(const Point& from, const Point& to) const;

    MDV_NODISCARD std::vector<Eigen::Vector3d> compute_vertex_normals() const noexcept;


    // Getters
    MDV_NODISCARD std::size_t num_vertices() const;
    MDV_NODISCARD std::size_t num_faces() const;

    MDV_NODISCARD std::string_view
                  name() const {
        return _name;
    };

    // clang-format off
    MDV_NODISCARD const Face&   face(const Index& id) const   { return _faces[id]; }
    MDV_NODISCARD const Vertex& vertex(const Index& id) const { return _vertices[id]; }
    MDV_NODISCARD Logger&       logger() const                { assert(_logger != nullptr); return *_logger.get(); };

    // clang-format on

    //  ___ _                 _
    // |_ _| |_ ___ _ __ __ _| |_ ___  _ __ ___
    //  | || __/ _ \ '__/ _` | __/ _ \| '__/ __|
    //  | || ||  __/ | | (_| | || (_) | |  \__ \
    // |___|\__\___|_|  \__,_|\__\___/|_|  |___/
    //

    // clang-format off
    MDV_NODISCARD Face::Iterator      faces_begin() noexcept       { return _faces.begin(); }
    MDV_NODISCARD Face::ConstIterator faces_cbegin() noexcept      { return _faces.cbegin(); }
    MDV_NODISCARD Face::ConstIterator faces_begin() const noexcept { return _faces.begin(); }
    MDV_NODISCARD Face::Iterator      faces_end() noexcept         { return _faces.end(); }
    MDV_NODISCARD Face::ConstIterator faces_cend() noexcept        { return _faces.cend(); }
    MDV_NODISCARD Face::ConstIterator faces_end() const noexcept   { return _faces.end(); }
    MDV_NODISCARD boost::iterator_range<Face::Iterator> faces() noexcept            { return {faces_begin(), faces_end()}; }
    MDV_NODISCARD boost::iterator_range<Face::ConstIterator> faces() const noexcept { return {faces_begin(), faces_end()}; }

    MDV_NODISCARD Vertex::Iterator      vertices_begin() noexcept       { return _vertices.begin(); }
    MDV_NODISCARD Vertex::ConstIterator vertices_cbegin() noexcept      { return _vertices.cbegin(); }
    MDV_NODISCARD Vertex::ConstIterator vertices_begin() const noexcept { return _vertices.begin(); }
    MDV_NODISCARD Vertex::Iterator      vertices_end() noexcept         { return _vertices.end(); }
    MDV_NODISCARD Vertex::ConstIterator vertices_cend() noexcept        { return _vertices.cend(); }
    MDV_NODISCARD Vertex::ConstIterator vertices_end() const noexcept   { return _vertices.end(); }
    MDV_NODISCARD boost::iterator_range<Vertex::Iterator> vertices() noexcept            { return {vertices_begin(), vertices_end()}; }
    MDV_NODISCARD boost::iterator_range<Vertex::ConstIterator> vertices() const noexcept { return {vertices_begin(), vertices_end()}; }

    // clang-format on

    // clang-format off
    MDV_NODISCARD CgalImpl&        cgal()       { assert(_impl); return *_impl; }
    MDV_NODISCARD const CgalImpl&  cgal() const { assert(_impl); return *_impl; }

    // clang-format on

    const Face& random_face() const;


private:
    Mesh(gsl::owner<CgalImpl*> data, const std::string& name);

    // Members
    Logger::SharedPtr     _logger = default_logger;
    gsl::owner<CgalImpl*> _impl   = nullptr;
    std::string           _name;
    Vertex::Vector        _vertices;
    Face::Vector          _faces;
    HalfEdge::Vector      _half_edges;

    Vertex&
    emplace_vertex(const CartesianPoint& pos) {
        return _vertices.emplace_back(*this, pos);
    }

    Face&
    emplace_face() {
        return _faces.emplace_back(*this);
    }

    HalfEdge&
    emplace_halfedge() {
        return _half_edges.emplace_back(*this);
    }

    void add_face(const IndexTriplet& v_ids);
    void construct_opposite_halfedges();
    void fill_halfedges();
    bool datastructure_correctly_initialised() const;
};

}  // namespace mdv::mesh


#endif  // MDV_MESH_CLASS_HPPmesh.hp
