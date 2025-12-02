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

    logger().info("Number of vertices: {}", cgal()._mesh.num_vertices());
    logger().info("Number of faces: {}", cgal()._mesh.num_faces());
}

Mesh::~Mesh() {
    logger().trace("Deleting mesh ptr at {}", static_cast<void*>(_impl));
    delete _impl;
}

Mesh::Mesh(Mesh&& other) :
        _logger(other._logger), _impl(other._impl), _name(std::move(other._name)) {
    logger().trace(
            "Moving mesh '{}' from {} to {}",
            name(),
            static_cast<void*>(&other),
            static_cast<void*>(this)
    );
    other._impl = nullptr;
}

Mesh&
Mesh::operator=(Mesh&& other) noexcept {
    this->_logger = std::move(other._logger);
    this->_impl   = other._impl;
    other._impl   = nullptr;
    this->_name   = std::move(other._name);
    return *this;
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
    using mdv::condition::are_orthogonal, mdv::condition::is_zero_norm;
    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    using Mat33 = Eigen::Matrix<double, 3, 3>;

    logger().debug(
            "Building geodesic from {} to {}",
            eigen_to_str(from.position()),
            eigen_to_str(to.position())
    );

    if (is_zero_norm(from.position() - to.position())) { return {}; }
    if (from.face() == to.face()) return {from.position(), to.position()};

    // TODO: reimplement this section of code to simplify computation of trivial geodesics
    // const HalfEdge* const he = from.face().adjacent_to(to.face());
    // const HalfEdge* const he = nullptr;

    // if (he != nullptr) {
    //     Mat33         base;
    //     long          n_checks = 0;
    //     HalfEdge*     to_he    = he->twin()->face().half_edge();
    //     const Vertex* v_modify = &he->twin()->prev()->origin();
    //     const Vec3    v0       = he->origin_position();
    //     const Vec3    n_from   = from.face().normal();

    //     for (long i = 0; i < 3; ++i) {
    //         Vec3 vi = to_he->origin_position();
    //         if (&to_he->origin() == v_modify) {
    //             const Vec3 delta_to   = to_he->origin_position() - v0;
    //             const Vec3 delta_from = he->aligning_rotation().inverse() * delta_to;
    //             vi                    = v0 + delta_from;
    //             ++n_checks;
    //         }

    //         assert(are_orthogonal(vi - v0, n_from));
    //         base.col(i) = vi;

    //         // step halfedge
    //         to_he = to_he->next();
    //     }
    //     assert(n_checks == 1);

    //     const Vec3 dest = base * to.barycentric();
    //     assert(are_orthogonal(dest - v0, n_from));

    //     const Edge             e1(*he);
    //     const Edge             e2 = Edge::from_positions(from.position(), dest);
    //     const EdgeIntersection intersection(e1, e2);
    //     assert(mdv::condition::is_zero(distance(*he, intersection.intersection_point)));
    //     assert(intersection.sols(0) < 1.0);
    //     assert(intersection.sols(0) > 0.0);

    //     return std::vector<Vec3>(
    //             {from.position(), intersection.intersection_point, to.position()}
    //     );
    // }


    Geodesic res = (*cgal()._geodesic_constructor)(from, to);

    assert(mdv::condition::are_equal(res.front(), from.position())
           && mdv::condition::are_equal(res.back(), to.position()));
    return res;
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

Face
Mesh::random_face() const {
    static std::random_device                  rand_dev;
    static std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<std::size_t> distribution(0, num_faces() - 1);
    return face(distribution(generator));
}

Eigen::MatrixXd
Mesh::get_vertex_matrix() const {
    Eigen::MatrixXd res(num_vertices(), 3);
    for (long i = 0; i < num_vertices(); ++i) res.row(i) = vertex(i).position();
    return res;
}

Eigen::MatrixXi
Mesh::get_face_matrix() const {
    Eigen::MatrixXi res(num_faces(), 3);
    for (long i = 0; i < num_faces(); ++i) {
        const auto [b0, b1, b2] = face(i).vertices_ids();
        res.row(i)              = Eigen::Vector3i({b0, b1, b2});
    }
    return res;
}

Eigen::MatrixXd
Mesh::get_face_matrix_double() const {
    const Eigen::MatrixXi mat = get_face_matrix();
    Eigen::MatrixXd       res(mat.rows(), mat.cols());
    for (long i = 0; i < mat.rows(); ++i)
        for (long j = 0; j < mat.cols(); ++j)
            res(i, j) = static_cast<double>(mat(i, j));
    return res;
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

Eigen::MatrixXd
Mesh::get_face_matrix_double() const {
    const Eigen::MatrixXi mat = get_face_matrix();
    Eigen::MatrixXd       res(mat.rows(), mat.cols());
    for (long i = 0; i < mat.rows(); ++i)
        for (long j = 0; j < mat.cols(); ++j)
            res(i, j) = static_cast<double>(mat(i, j));
    return res;
}
