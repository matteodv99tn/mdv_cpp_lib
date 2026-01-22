#include "mdv/mesh/mesh.hpp"

#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <cstdint>
#include <Eigen/Geometry>
#include <gsl/assert>
#include <random>

#include <range/v3/all.hpp>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/helpers.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"


// \cond DOXYGEN_IGNORE
using mdv::mesh::Face;
using mdv::mesh::Mesh;
using mdv::mesh::Vertex;
using mdv::mesh::internal::CgalImpl;
using std::filesystem::path;

namespace rs = ::ranges;
namespace rv = ::ranges::views;
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
    using Kernel = internal::CgalImpl::Kernel;

    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    using Mat33 = Eigen::Matrix<double, 3, 3>;

    logger().debug(
            "Building geodesic from {} to {}",
            eigen_to_str(from.position()),
            eigen_to_str(to.position())
    );

    if (is_zero_norm(from.position() - to.position())) return {};

    const auto from_id = internal::to_face_impl(from.face());
    const auto to_id   = internal::to_face_impl(to.face());
    if (from_id == to_id) return {from.position(), to.position()};


    if (from.get_as<Point::PointInFaceDescriptor>() != nullptr
        && to.get_as<Point::PointInFaceDescriptor>() != nullptr) {
        const auto& m         = internal::get_mesh_impl(from.mesh());
        const auto  shared_he = internal::shared_halfedge(from_id, to_id, m);

        if (shared_he.has_value()) {
            const auto               he       = shared_he.value();
            const Eigen::Quaterniond rotation = internal::relative_face_rotation(he, m);

            const auto  p0_c      = m.point(source(he, m));
            const auto  p1_c      = m.point(target(he, m));
            const Vec3d p0        = internal::convert(p0_c);
            const Vec3d p1        = internal::convert(p1_c);
            const Vec3d delta     = to.position() - p0;
            const Vec3d delta_rot = rotation * delta;

            const Vec3d             p_star = p0 + delta_rot;
            const Kernel::Segment_3 segment(p0_c, p1_c);
            const Kernel::Ray_3     ray(
                    internal::point3_from_eigen(from.position()),
                    internal::vector3_from_eigen(p_star - from.position())
            );
            auto midpt = internal::edge_ray_intersection(segment, ray);

            if (!midpt.has_value()) {
                throw std::runtime_error(
                        "Expected ray intersection, but intersection not found!"
                );
            }

            return {from.position(), internal::convert(midpt.value()), to.position()};
        }
    }

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

Vertex
Mesh::closest_vertex(const CartesianPoint& pt) {
    const auto [_, f_id] = cgal()._aabb_tree.closest_point_and_primitive(
            internal::point3_from_eigen(pt)
    );
    const auto& m    = internal::get_mesh_impl(*this);
    auto        func = [&pt, &m](const auto id) -> double {
        return (internal::convert(m.point(id)) - pt).norm();
    };

    const auto vids = CGAL::vertices_around_face(halfedge(f_id, m), m) | rs::to_vector;
    const auto min_id = rs::min_element(vids, std::less{}, func);
    return Vertex{*this, *min_id};
}

Mesh
Mesh::extract_normal_bounded_surface(
        const Mesh& mesh, const Point& pt, const double max_normal_angle
) {
    enum class VisitState : std::uint8_t {
        UNVISITED,
        VALID_FACE,
        INVALID_FACE,
    };

    std::vector<VisitState> face_class(mesh.num_faces(), VisitState::UNVISITED);

    const auto   n_ref     = pt.face().normal();
    const double cos_angle = std::cos(max_normal_angle * M_PI / 180.0);

    const auto classify = [&cos_angle, &n_ref](const Face& f) -> VisitState {
        const auto n = f.normal();
        if (n.dot(n_ref) > cos_angle) return VisitState::VALID_FACE;
        return VisitState::INVALID_FACE;
    };

    // Recursive lambda
    // https://stackoverflow.com/questions/78166176/how-can-i-write-an-inline-recursive-lambda-in-c
    const auto propagate = [&](const auto& self, const Index& id) -> void {
        if (face_class[id] != VisitState::UNVISITED) return;

        const auto f                     = mesh.face(id);
        face_class[id]                   = classify(f);
        if(face_class[id] == VisitState::INVALID_FACE) return;

        const auto [f_id1, f_id2, f_id3] = f.neighbour_ids();
        self(self, f_id1);
        self(self, f_id2);
        self(self, f_id3);
    };

    mesh.logger().info(
            "Constructing submesh by from point {} - maximum normal angle: {}deg",
            pt.describe(),
            max_normal_angle
    );
    propagate(propagate, pt.face().id());

    const auto& m_original = internal::get_mesh_impl(mesh);
    auto        m_new      = m_original;

    long n_removed = 0;
    for (long i = 0; i < mesh.num_faces(); ++i) {
        if (face_class[i] != VisitState::VALID_FACE) {
            ++n_removed;
            CgalImpl::CgalFaceIndex f_id(i);
            CGAL::Euler::remove_face(CGAL::halfedge(f_id, m_new), m_new);
        }
    }
    mesh.logger().debug("Removed {} faces", n_removed);

    CGAL::Polygon_mesh_processing::remove_isolated_vertices(m_new);
    m_new.collect_garbage();
    CGAL::Polygon_mesh_processing::remove_isolated_vertices(m_new);

    Logger::SharedPtr logger = mesh._logger;
    return {
            new CgalImpl(std::move(m_new), std::move(logger)),
            fmt::format("{}_normal_bounded", mesh.name()),
    };
}
