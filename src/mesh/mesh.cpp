#include "mdv/mesh/mesh.hpp"

#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <Eigen/Geometry>
#include <gsl/assert>
#include <random>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/helpers.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"


// \cond DOXYGEN_IGNORE
using mdv::mesh::Face;
using mdv::mesh::Mesh;
using mdv::mesh::Vertex;
using mdv::mesh::internal::CgalImpl;
using std::filesystem::path;
// \endcond

mdv::Logger::SharedPtr Mesh::default_logger = get_default_logger();

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//
Mesh
Mesh::from_file(const std::filesystem::path& file_path) {
    const std::string     file_name = file_path.stem().string();
    auto                  logger    = Mesh::default_logger;
    gsl::owner<CgalImpl*> data      = CgalImpl::from_file(file_path, std::move(logger));
    return Mesh(data, file_name);
}

Mesh::Mesh(gsl::owner<CgalImpl*> cgal_data, const std::string& name) :
        _impl(cgal_data), _name(name) {
    Expects(cgal_data != nullptr);
    Expects(cgal_data->_logger != nullptr);

    _logger = cgal_data->_logger;

    build_halfedges();

    logger().info("Number of vertices: {}", cgal()._mesh.num_vertices());
    logger().info("Number of faces: {}", cgal()._mesh.num_faces());
}

Mesh::~Mesh() {
    logger().trace("Deleting mesh ptr at {}", static_cast<void*>(_impl));
    delete _impl;
}

Mesh::Mesh(Mesh&& other) :
        _logger(other._logger),
        _impl(other._impl),
        _name(std::move(other._name)),
        _vertices(std::move(other._vertices)),
        _faces(std::move(other._faces)),
        _half_edges(std::move(other._half_edges)) {
    logger().trace(
            "Moving mesh '{}' from {} to {}",
            name(),
            static_cast<void*>(&other),
            static_cast<void*>(this)
    );
    update_mesh_element_references();
    other._impl = nullptr;
}

Mesh&
Mesh::operator=(Mesh&& other) noexcept {
    this->_logger     = std::move(other._logger);
    this->_impl       = other._impl;
    other._impl       = nullptr;
    this->_name       = std::move(other._name);
    this->_vertices   = std::move(other._vertices);
    this->_faces      = std::move(other._faces);
    this->_half_edges = std::move(other._half_edges);
    update_mesh_element_references();
    return *this;
}

void
Mesh::update_mesh_element_references() {
    for (auto& f : _faces) f._mesh_ptr = this;
    for (auto& v : _vertices) v._mesh_ptr = this;
    for (auto& he : _half_edges) he._mesh_ptr = this;
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

void
Mesh::transform(const Eigen::Affine3d& transformation) {
    logger().info("Applying transformation to mesh");
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

    const Mesh::CgalImpl::Transform transform(
            m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, 1.0
    );
    CGAL::Polygon_mesh_processing::transform(transform, cgal()._mesh);
}

mdv::mesh::Geodesic
Mesh::build_geodesic(const Point& from, const Point& to) const {
    using mdv::condition::are_orthogonal;
    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    using Mat33 = Eigen::Matrix<double, 3, 3>;

    logger().debug(
            "Building geodesic from {} to {}",
            eigen_to_str(from.position()),
            eigen_to_str(to.position())
    );

    if (from.face() == to.face()) return {from.position(), to.position()};

    const HalfEdge* const he = from.face().adjacent_to(to.face());

    if (he != nullptr) {
        Mat33         base;
        long          n_checks = 0;
        HalfEdge*     to_he    = he->twin()->face().half_edge();
        const Vertex* v_modify = &he->twin()->prev()->origin();
        const Vec3    v0       = he->origin_position();
        const Vec3    n_from   = from.face().normal();

        for (long i = 0; i < 3; ++i) {
            Vec3 vi = to_he->origin_position();
            if (&to_he->origin() == v_modify) {
                const Vec3 delta_to   = to_he->origin_position() - v0;
                const Vec3 delta_from = he->aligning_rotation().inverse() * delta_to;
                vi                    = v0 + delta_from;
                ++n_checks;
            }

            assert(are_orthogonal(vi - v0, n_from));
            base.col(i) = vi;

            // step halfedge
            to_he = to_he->next();
        }
        assert(n_checks == 1);

        const Vec3 dest = base * to.barycentric();
        assert(are_orthogonal(dest - v0, n_from));

        const Edge             e1(*he);
        const Edge             e2 = Edge::from_positions(from.position(), dest);
        const EdgeIntersection intersection(e1, e2);
        assert(mdv::condition::is_zero(distance(*he, intersection.intersection_point)));
        assert(intersection.sols(0) < 1.0);
        assert(intersection.sols(0) > 0.0);

        return std::vector<Vec3>(
                {from.position(), intersection.intersection_point, to.position()}
        );
    }

    return (*cgal()._geodesic_constructor)(from, to);
}

//   ____      _   _
//  / ___| ___| |_| |_ ___ _ __ ___
// | |  _ / _ \ __| __/ _ \ '__/ __|
// | |_| |  __/ |_| ||  __/ |  \__ \
//  \____|\___|\__|\__\___|_|  |___/
//

std::size_t
Mesh::num_vertices() const {
    return cgal()._mesh.num_vertices();
}

std::size_t
Mesh::num_faces() const {
    return cgal()._mesh.num_faces();
}

void
Mesh::build_halfedges() {
    logger().debug("Synching half-edge data structure");
    logger().trace("Constructing half-edges vertices data");
    const auto eigen_vertices = _impl->yield_vertices();
    _vertices.reserve(eigen_vertices.size());
    for (const auto& v_pos : eigen_vertices) emplace_vertex(v_pos);


    logger().trace("Constructing half-edges face data");
    const auto eigen_faces = _impl->yield_faces();
    _half_edges.reserve(6 * eigen_faces.size());
    _faces.reserve(eigen_faces.size());
    for (const auto& face_vertex_ids : eigen_faces) add_face(face_vertex_ids);

    logger().trace("Constructing half-edges opposite edge pairs");
    construct_opposite_halfedges();
    logger().trace("Filling halfedges information");
    fill_halfedges();

    assert(datastructure_correctly_initialised());

    logger().trace("Baking vertex properties");
    for (auto& v : vertices()) v.bake_properties();

    logger().trace("Baking face properties");
    for (auto& f : faces()) f.bake_properties();
}

void
Mesh::add_face(const IndexTriplet& v_ids) {
    auto& f = emplace_face();

    auto& he1 = emplace_halfedge();
    auto& he2 = emplace_halfedge();
    auto& he3 = emplace_halfedge();

    // Assign face first half-edge
    f._he = &he1;

    // Assign created half-edges reference to the same face, and set vertices
    he1._face   = &f;
    he2._face   = &f;
    he3._face   = &f;
    he1._origin = &_vertices[v_ids[0]];
    he2._origin = &_vertices[v_ids[1]];
    he3._origin = &_vertices[v_ids[2]];

    // Create relationship between half-edges
    he1._next = &he2;
    he1._prev = &he3;

    he2._next = &he3;
    he2._prev = &he1;

    he3._next = &he1;
    he3._prev = &he2;

    // Assign half-edges to their origins (if they have not been mapped)
    if (he1._origin->_he == nullptr) he1._origin->_he = &he1;
    if (he2._origin->_he == nullptr) he2._origin->_he = &he2;
    if (he3._origin->_he == nullptr) he3._origin->_he = &he3;
}

void
Mesh::construct_opposite_halfedges() {
    for (std::size_t i = 0; i < _half_edges.size(); ++i) {
        auto& he_i = _half_edges[i];
        for (std::size_t j = i + 1; j < _half_edges.size(); ++j) {
            auto& he_j = _half_edges[j];

            if (he_i.is_opposite_of(he_j)) {
                assert(he_i._twin == nullptr);
                assert(he_j._twin == nullptr);
                he_i._twin = &he_j;
                he_j._twin = &he_i;
            }
        }
    }
}

void
Mesh::fill_halfedges() {
    const std::size_t he_size = _half_edges.size();
    for (std::size_t i = 0; i < he_size; ++i) {
        auto& he = _half_edges[i];
        if (he._twin != nullptr) continue;

        // Create opposite half-edge
        auto& twin_he = emplace_halfedge();
        he._twin      = &twin_he;

        // Fill opposite half-edge data
        twin_he._origin = he._next->_origin;
        twin_he._twin   = &he;
        assert(twin_he._next == nullptr);
        assert(twin_he._prev == nullptr);
        assert(twin_he._face == nullptr);
    }

    for (std::size_t i = he_size; i < _half_edges.size(); ++i) {
        auto& he_i = _half_edges[i];
        assert(he_i._twin);
        assert(he_i._face == nullptr);
        assert(he_i._origin != nullptr);

        // for (std::size_t j = i + 1; j < _half_edges.size(); ++j) {
        for (std::size_t j = he_size; j < _half_edges.size(); ++j) {
            auto& he_j = _half_edges[j];

            if (he_i._origin == he_j._twin->_origin) {
                assert(he_i._next == nullptr);
                assert(he_j._prev == nullptr);
                he_i._next = &he_j;
                he_j._prev = &he_i;
            }
        }
    }
}

const Face&
Mesh::random_face() const {
    static std::random_device                  rand_dev;
    static std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<std::size_t> distribution(0, num_faces() - 1);
    return _faces[distribution(generator)];
}

bool
Mesh::datastructure_correctly_initialised() const {
    // Check vertices
    for (const auto& v : _vertices)
        if (v._he == nullptr) return false;

    // Check faces
    for (const auto& f : _faces)
        if (f._he == nullptr) return false;

    // Check half-edges
    for (const auto& he : _half_edges) {
        const bool c1 = he._origin == nullptr;
        const bool c2 = he._twin == nullptr;
        const bool c3 = he._next == nullptr;
        const bool c4 = he._prev == nullptr;
        if (c1 || c2 || c3 || c4) {
            // Func
            return false;
        }
    }

    return true;
}

Eigen::MatrixXd
Mesh::get_vertex_matrix() const {
    Eigen::MatrixXd res(num_vertices(), 3);
    for (long i = 0; i < num_vertices(); ++i) res.row(i) = _vertices[i].position();
    return res;
}

Eigen::MatrixXi
Mesh::get_face_matrix() const {
    Eigen::MatrixXi res(num_faces(), 3);
    for (long i = 0; i < num_faces(); ++i) {
        const Face&     f  = _faces[i];
        const HalfEdge* he = f.half_edge();
        const int       f0 = he->origin().id();
        he                 = he->next();
        const int f1       = he->origin().id();
        he                 = he->next();
        const int f2       = he->origin().id();
        res.row(i)         = Eigen::Vector3i{f0, f1, f2};
    }
    return res;
}
