#include <algorithm>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <Eigen/Geometry>
#include <execution>
#include <spdlog/spdlog.h>

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/utils/logging.hpp>

#include "cgal_data.hpp"

using mdv::mesh::Mesh;

using std::filesystem::path;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//
Mesh::Mesh(const path& file_path) : _logger(nullptr), _file_path(file_path) {
    _logger    = class_logger_factory("Mesh", file_name(), Trace);
    _cgal_data = new CgalMesh(file_path, _logger);
    Ensures(_cgal_data != nullptr);

    _logger->info("Loaded mesh from file {}", file_path.string());
    _logger->info("Number of vertices: {}", _cgal_data->_mesh.num_vertices());
    _logger->info("Number of faces: {}", _cgal_data->_mesh.num_faces());

    sync_vertex_data();
    sync_face_data();

    _logger->trace("Finding neighbouring faces");
    _neighbouring_faces.reserve(_cgal_data->_mesh.num_faces());
    auto is_neighbour = [](const std::array<long, 3>& face,
                           const long&                v1_id,
                           const long&                v2_id) -> bool {
        const bool v1_found = std::find(face.begin(), face.end(), v1_id) != face.end();
        const bool v2_found = std::find(face.begin(), face.end(), v2_id) != face.end();
        return v1_found && v2_found;
    };
    auto find_face_neighbour = [this, is_neighbour](const long& face_id) -> void {
        const std::array<long, 3>& curr_face   = _f_mat[face_id];
        std::array<long, 3>        face_neighs = {-1, -1, -1};

        for (long i = 0; i < num_faces(); ++i) {
            if (i == face_id) continue;
            const std::array<long, 3>& f = _f_mat[i];

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
}

Mesh::CgalMesh::CgalMesh(const path& file_path, LoggerPtr_t& logger) :
        _logger(logger), _shortest_path(nullptr) {
    const bool loaded = CGAL::IO::read_polygon_mesh(file_path.string(), _mesh);
    if (!loaded) {
        _logger->error("Unable to load mesh {}", file_path.string());
        throw std::runtime_error("Cannot load mesh");
    }

    _shortest_path = std::make_unique<ShortestPath>(_mesh);
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

    const Mesh::CgalMesh::Transform transform(
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

std::size_t
Mesh::num_vertices() const {
    return _cgal_data->_mesh.num_vertices();
}

std::size_t
Mesh::num_faces() const {
    return _cgal_data->_mesh.num_faces();
}

Mesh::FaceIterator
Mesh::faces_begin() const noexcept {
    return {this, 0};
}

Mesh::FaceIterator
Mesh::faces_end() const noexcept {
    return {this, static_cast<long>(num_faces())};
}

boost::iterator_range<Mesh::FaceIterator>
Mesh::faces() const noexcept {
    return {faces_begin(), faces_end()};
}

Mesh::VertexIterator
Mesh::vertices_begin() const noexcept {
    return {this, 0};
}

Mesh::VertexIterator
Mesh::vertices_end() const noexcept {
    return {this, static_cast<long>(num_vertices())};
}

boost::iterator_range<Mesh::VertexIterator>
Mesh::vertices() const noexcept {
    return {vertices_begin(), vertices_end()};
}

//  ____       _            _
// |  _ \ _ __(_)_   ____ _| |_ ___  ___
// | |_) | '__| \ \ / / _` | __/ _ \/ __|
// |  __/| |  | |\ V / (_| | ||  __/\__ \
// |_|   |_|  |_| \_/ \__,_|\__\___||___/
//
void
Mesh::sync_vertex_data() {
    _logger->trace("Syncing matrix V");
    _v_mat.resize(3, static_cast<long>(num_vertices()));
    for (const CgalMesh::Mesh::Vertex_index& vertex : _cgal_data->_mesh.vertices()) {
        const auto cgal_vertex = _cgal_data->_mesh.point(vertex);
        _v_mat.col(vertex.idx()) =
                Eigen::Vector3d(cgal_vertex.x(), cgal_vertex.y(), cgal_vertex.z());
    }
}

void
Mesh::sync_face_data() {
    _logger->trace("Syncing matrix F");
    _f_mat.resize(static_cast<long>(num_faces()));
    for (const CgalMesh::Mesh::Face_index& face : _cgal_data->_mesh.faces()) {
        const auto vertex_iter = CGAL::vertices_around_face(
                _cgal_data->_mesh.halfedge(face), _cgal_data->_mesh
        );
        int i = 0;
        for (const auto& vi : vertex_iter) {
            Expects(i < 3);
            _f_mat[static_cast<long>(face)].at(i) = vi.idx();
            i++;
        }
    }
}
