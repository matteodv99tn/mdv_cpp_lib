#include <algorithm>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>
#include <thread>

#include <mdv/mesh/fwd.hpp>
#include <mdv/mesh/mesh.hpp>
#include <mdv/utils/logging.hpp>
#include <mdv/utils/logging_extras.hpp>

#include "cgal_data.hpp"

using mdv::mesh::Mesh;

using std::filesystem::path;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//
Mesh::Mesh(const path& file_path) : _file_path(file_path) {
    _logger = class_logger_factory("Mesh", file_name(), Debug);
    // NOLINTNEXTLINE can't initialise in member initialisation list due to ordering
    _cgal_data = new CgalMesh(file_path, _logger);
    Ensures(_cgal_data != nullptr);

    _logger->info("Loaded mesh from file {}", file_path.string());
    _logger->info("Number of vertices: {}", _cgal_data->_mesh.num_vertices());
    _logger->info("Number of faces: {}", _cgal_data->_mesh.num_faces());

    sync_vertex_data();
    sync_face_data();
    _cgal_data->build_vertex_normals_map();

    _logger->debug("Finding neighbouring faces");
    _neighbouring_faces.resize(_cgal_data->_mesh.num_faces());
    // Helper function that checks if two vertices are neighbours of a given face
    auto is_neighbour = [](const std::array<long, 3>& face,
                           const long&                v1_id,
                           const long&                v2_id) -> bool {
        const bool v1_found = std::find(face.begin(), face.end(), v1_id) != face.end();
        const bool v2_found = std::find(face.begin(), face.end(), v2_id) != face.end();
        return v1_found && v2_found;
    };
    // Helper function that find the neighbouring faces of a given face
    auto find_face_neighbour = [this](const Face::Index& face_id) -> void {
        std::array<long, 3> face_neighs = {-1, -1, -1};
        int                 i           = 0;

        // Use CGAL's halfedge circulator to find adjacent faces
        const auto half_edge = _cgal_data->_mesh.halfedge(
                _cgal_data->_mesh.face(static_cast<CGAL::SM_Halfedge_index>(face_id))
        );
        for (const auto edge : _cgal_data->_mesh.halfedges_around_face(half_edge)) {
            const auto opposite = _cgal_data->_mesh.opposite(edge);
            if (!_cgal_data->_mesh.is_border(opposite))
                face_neighs[i] = _cgal_data->_mesh.face(opposite).idx();
            i++;
        }
        _neighbouring_faces.at(face_id) = face_neighs;
    };

    // Helper function to find neighbours in batches (to enable efficient parallelism)
    auto batch_find_process_neighbours =
            [this, find_face_neighbour](const long& start, const long& end) -> void {
        for (auto i = start; i < end; ++i) { find_face_neighbour(i); }
    };


    // Allocate memory for neighbouring faces and populate them
    _neighbouring_faces.resize(num_faces());
    const auto                batch_size = 1000;
    std::vector<std::jthread> threads;
    for (Face::Index i = 0; i * batch_size < num_faces(); i++) {
        // threads.emplace_back(find_face_neighbour, i);
        const auto start_id = i * batch_size;
        const auto end_id   = (i + 1) * batch_size;
        threads.emplace_back(
                batch_find_process_neighbours,
                start_id,
                std::min(end_id, static_cast<Face::Index>(num_faces()))
        );
    }
}

Mesh::CgalMesh::CgalMesh(const path& file_path, SpdLoggerPtr& logger) :
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

Mesh::Geodesic
Mesh::build_geodesic(const Point& from, const Point& to, GeodesicBuilderPolicy policy)
        const {
    auto internal_state_implementation = [this, &from, &to]() -> Geodesic {
        const auto curr_target = _cgal_data->_current_shortpath_source.value_or(
                Mesh::Point::undefined(to.mesh())
        );
        const bool is_close = curr_target.face_id() == to.face_id()
                              && (curr_target.position() - to.position()).norm() < 1e-3;

        if (!is_close) {
            _logger->trace("Updating shortest path source point");
            if (_cgal_data->_current_shortpath_source.has_value())
                _cgal_data->_shortest_path->remove_all_source_points();
            _cgal_data->_shortest_path->add_source_point(location_from_mesh_point(to));
            _cgal_data->_current_shortpath_source = to;
        }
        return construct_geodesic(*_cgal_data->_shortest_path, from);
    };

    auto exec_local_implementation = [](const Point& from,
                                        const Point& to) -> Geodesic {
        Mesh::CgalMesh::ShortestPath shpathobj(from.mesh()._cgal_data->_mesh);
        shpathobj.add_source_point(location_from_mesh_point(to));
        return construct_geodesic(shpathobj, from);
    };

    _logger->debug(
            "Building geodesic from {} to {}",
            eigen_to_str(from.position()),
            eigen_to_str(to.position())
    );
    switch (policy) {
        case InternalState:
            _logger->trace("Computing geodesic in InternalState mode");
            return internal_state_implementation();
        case ExecutionLocal:
            _logger->trace("Computing geodesic in ExecutionLocal mode");
            return exec_local_implementation(from, to);
    }
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
    _v_mat.resize(num_vertices());
    for (const CgalMesh::Mesh::Vertex_index& vertex : _cgal_data->_mesh.vertices()) {
        const auto cgal_vertex = _cgal_data->_mesh.point(vertex);
        _v_mat[vertex.idx()] =
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
