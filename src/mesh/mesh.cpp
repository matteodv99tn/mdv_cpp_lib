#include <algorithm>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <Eigen/Geometry>
#include <execution>
#include <spdlog/spdlog.h>

#include <mdv/mesh/face.hpp>
#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/mesh/vertex.hpp>
#include <mdv/utils/logging.hpp>

#include "cgal_data.hpp"

using mdv::mesh::CgalData;
using mdv::mesh::ConstFaceIterator;
using mdv::mesh::ConstVertexIterator;
using mdv::mesh::Face;
using mdv::mesh::FaceIndex_t;
using mdv::mesh::FaceIterator;
using mdv::mesh::Mesh;
using mdv::mesh::Vertex;
using mdv::mesh::VertexIndex_t;
using mdv::mesh::VertexIterator;

using std::filesystem::path;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//
Mesh::Mesh(const path& file_path) : _logger(nullptr), _file_path(file_path) {
    _logger    = class_logger_factory("Mesh", file_name(), Trace);
    _cgal_data = new CgalData(file_path, _logger);
    Ensures(_cgal_data != nullptr);

    _logger->info("Loaded mesh from file {}", file_path.string());
    _logger->info("Number of vertices: {}", _cgal_data->_mesh.num_vertices());
    _logger->info("Number of faces: {}", _cgal_data->_mesh.num_faces());


    _v_mat.resize(3, _cgal_data->_mesh.num_vertices());
    _f_mat.resize(3, _cgal_data->_mesh.num_faces());

    _logger->trace("Building matrix V");
    for (const CgalData::Mesh_t::Vertex_index& vertex : _cgal_data->_mesh.vertices()) {
        const auto cgal_vertex = _cgal_data->_mesh.point(vertex);
        _v_mat.col(vertex.idx()) =
                Eigen::Vector3d(cgal_vertex.x(), cgal_vertex.y(), cgal_vertex.z());
    }

    _logger->trace("Building matrix F");
    for (const CgalData::Mesh_t::Face_index& face : _cgal_data->_mesh.faces()) {
        const auto vertex_iter = CGAL::vertices_around_face(
                _cgal_data->_mesh.halfedge(face), _cgal_data->_mesh
        );
        int i = 0;
        for (const auto& vi : vertex_iter) {
            Expects(i < 3);
            _f_mat(i, static_cast<long>(face)) = vi.idx();
            i++;
        }
    }

    _logger->debug("Finding neighbouring faces");
    _neighbouring_faces.reserve(_cgal_data->_mesh.num_faces());
#if 1
    std::transform(
            faces_begin(),
            faces_end(),
            _neighbouring_faces.begin(),
            [](const Face& f) { return f.neighbour_faces_ids(); }
    );
#else

    auto is_neighbour = [](const std::array<long, 3>& face,
                           const long&                v1_id,
                           const long&                v2_id) -> bool {
        const bool v1_found = std::find(face.begin(), face.end(), v1_id) != face.end();
        const bool v2_found = std::find(face.begin(), face.end(), v2_id) != face.end();
        return v1_found && v2_found;
    };

    auto find_face_neighbour = [this, is_neighbour](const long& face_id) -> void {
        const std::array<long, 3> curr_face = {
                _f_mat(0, face_id), _f_mat(1, face_id), _f_mat(2, face_id)
        };
        std::array<long, 3> face_neighs = {-1, -1, -1};

        for (long i = 0; i < num_faces(); ++i) {
            if (i == face_id) continue;
            const std::array<long, 3> f = {
                    _f_mat(0, i), _f_mat(1, i), _f_mat(2, i)
            };

            if (is_neighbour(f, curr_face.at(0), curr_face.at(1))) {
                assert(face_neighs.at(0) == -1);
                face_neighs.at(0) = i;
            }
            if (is_neighbour(f, curr_face.at(0), curr_face.at(2))) {
                assert(face_neighs.at(1) == -1);
                face_neighs.at(1) = i;
            }
            if (is_neighbour(f, curr_face.at(1), curr_face.at(2))) {
                assert(face_neighs.at(2) == -1);
                face_neighs.at(2) = i;
            }
        }
        _neighbouring_faces.at(face_id) = face_neighs;
    };
#endif
}

CgalData::CgalData(const path& file_path, LoggerPtr_t& logger) :
        _logger(logger), _shortest_path(nullptr) {
    const bool loaded = CGAL::IO::read_polygon_mesh(file_path.string(), _mesh);
    if (!loaded) {
        _logger->error("Unable to load mesh {}", file_path.string());
        throw std::runtime_error("Cannot load mesh");
    }

    _shortest_path = std::make_unique<ShortestPath_t>(_mesh);
    _logger->trace("Initialised shortest path object");

    _shortest_path->build_aabb_tree(_aabb_tree);
    _logger->trace("Built AABB tree of the mesh");
}

Mesh::~Mesh() {
    _logger->trace("Releasing CGAL mesh");
    delete _cgal_data;
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

void
Mesh::transform(const Eigen::Affine3d& transformation) {
    _logger->info("Applying transformation to mesh");
    const double m11 = transformation(0, 0);
    const double m12 = transformation(0, 1);
    const double m13 = transformation(0, 2);
    const double m14 = transformation(0, 3);
    const double m21 = transformation(1, 0);
    const double m22 = transformation(1, 1);
    const double m23 = transformation(1, 2);
    const double m24 = transformation(1, 3);
    const double m31 = transformation(2, 0);
    const double m32 = transformation(2, 1);
    const double m33 = transformation(2, 2);
    const double m34 = transformation(2, 3);

    const CgalData::Transform_t transform(
            m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, 1.0
    );
    CGAL::Polygon_mesh_processing::transform(transform, _cgal_data->_mesh);
}

//   ____      _   _
//  / ___| ___| |_| |_ ___ _ __ ___
// | |  _ / _ \ __| __/ _ \ '__/ __|
// | |_| |  __/ |_| ||  __/ |  \__ \
//  \____|\___|\__|\__\___|_|  |___/
//
std::string
Mesh::file_name() const {
    return _file_path.stem().string();
}

VertexIndex_t
Mesh::num_vertices() const {
    return _cgal_data->_mesh.num_vertices();
}

FaceIndex_t
Mesh::num_faces() const {
    return _cgal_data->_mesh.num_vertices();
}

Vertex
Mesh::vertex(const VertexIndex_t& id) {
    return {this, id};
}

Vertex
Mesh::vertex(const VertexIndex_t& id) const {
    return {this, id};
}

Face
Mesh::face(const FaceIndex_t& id) {
    return {this, id};
}

Face
Mesh::face(const FaceIndex_t& id) const {
    return {this, id};
}

VertexIterator
Mesh::vertices_begin()  noexcept{
    return {this, 0};
}

VertexIterator
Mesh::vertices_end()  noexcept{
    return {this, num_vertices()};
}

ConstVertexIterator
Mesh::cvertices_begin() const  noexcept{
    return {const_cast<const Mesh*>(this), 0};
}

ConstVertexIterator
Mesh::cvertices_end() const  noexcept{
    return {const_cast<const Mesh*>(this), num_vertices()};
}

ConstVertexIterator
Mesh::vertices_begin() const  noexcept{
    return {this, 0};
}

ConstVertexIterator
Mesh::vertices_end() const  noexcept{
    return {this, num_vertices()};
}

boost::iterator_range<VertexIterator>
Mesh::vertices()  noexcept{
    return {vertices_begin(), vertices_end()};
}

boost::iterator_range<ConstVertexIterator>
Mesh::vertices() const  noexcept{
    return boost::make_iterator_range(cvertices_begin(), cvertices_end());
}

boost::iterator_range<ConstVertexIterator>
Mesh::cvertices() const  noexcept{
    return {cvertices_begin(), cvertices_end()};
}

FaceIterator
Mesh::faces_begin()  noexcept{
    return {this, 0};
}

FaceIterator
Mesh::faces_end()  noexcept{
    return {this, num_faces()};
}

ConstFaceIterator
Mesh::cfaces_begin() const  noexcept{
    return {const_cast<const Mesh*>(this), 0};
}

ConstFaceIterator
Mesh::cfaces_end() const  noexcept{
    return {const_cast<const Mesh*>(this), num_faces()};
}

ConstFaceIterator
Mesh::faces_begin() const  noexcept{
    return {this, 0};
}

ConstFaceIterator
Mesh::faces_end() const  noexcept{
    return {this, num_faces()};
}

boost::iterator_range<FaceIterator>
Mesh::faces()  noexcept{
    return {faces_begin(), faces_end()};
}

boost::iterator_range<ConstFaceIterator>
Mesh::faces() const  noexcept{
    return {faces_begin(), faces_end()};
}

boost::iterator_range<ConstFaceIterator>
Mesh::cfaces() const  noexcept{
    return {cfaces_begin(), cfaces_end()};
}
