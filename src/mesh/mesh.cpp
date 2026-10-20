#include "mdv/mesh/mesh.hpp"

#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <cstdint>
#include <Eigen/Geometry>
#include <gsl/assert>
#include <random>
#include <utility>

#include <range/v3/all.hpp>

#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/vertex.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"

namespace mdv::mesh {

Logger::SharedPtr Mesh::default_logger = get_default_logger();

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
    return {data, file_name};
}

Mesh::Mesh(gsl::owner<CgalImpl*> cgal_data, std::string name) :
        _logger(cgal_data != nullptr ? cgal_data->_logger : nullptr),
        _impl(cgal_data),
        _name(std::move(name)) {
    Expects(cgal_data != nullptr);
    Expects(cgal_data->_logger != nullptr);

    logger().info("Number of vertices: {}", cgal()._mesh.num_vertices());
    logger().info("Number of faces: {}", cgal()._mesh.num_faces());
}

Mesh::~Mesh() {
    if (_impl != nullptr)
        logger().trace("Deleting mesh ptr at {}", static_cast<void*>(_impl));
    delete _impl;
}

Mesh::Mesh(Mesh&& other) noexcept :
        _logger(std::move(other._logger)),
        _impl(std::exchange(other._impl, nullptr)),
        _name(std::move(other._name)) {
}

Mesh&
Mesh::operator=(Mesh&& other) noexcept {
    if (this == &other) return *this;
    delete _impl;
    _logger = std::move(other._logger);
    // std::exchange transfers ownership, which the owner-alias check cannot infer.
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
    _impl = std::exchange(other._impl, nullptr);
    _name = std::move(other._name);
    return *this;
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

void
Mesh::scale(const double factor) {
    logger().info("Scaling mesh by factor {}", factor);
    cgal().scale(factor);
}

void
Mesh::transform(const Eigen::Affine3d& transformation) {
    logger().info("Applying transformation to mesh");
    cgal().transform(transformation);
}

Geodesic
build_geodesic(const Mesh& mesh, const Point& from, const Point& to) {
    using mdv::condition::are_orthogonal, mdv::condition::is_zero_norm;
    using Kernel = internal::CgalImpl::Kernel;

    using Vec2  = Eigen::Vector2d;
    using Vec3  = Eigen::Vector3d;
    using Mat32 = Eigen::Matrix<double, 3, 2>;
    using Mat33 = Eigen::Matrix<double, 3, 3>;

    mesh.logger().debug(
            "Building geodesic from {} to {}",
            eigen_to_str(position(mesh, from)),
            eigen_to_str(position(mesh, to))
    );

    if (is_zero_norm(position(mesh, from) - position(mesh, to))) return {};

    const auto from_id = internal::to_cgal(face(mesh, from));
    const auto to_id   = internal::to_cgal(face(mesh, to));
    if (from_id == to_id) return {position(mesh, from), position(mesh, to)};


    if (from.get_as<PointInFace>() != nullptr && to.get_as<PointInFace>() != nullptr) {
        const auto& m         = internal::cgal_mesh(mesh);
        const auto  shared_he = internal::shared_halfedge(from_id, to_id, m);

        if (shared_he.has_value()) {
            const auto               he       = shared_he.value();
            const Eigen::Quaterniond rotation = internal::relative_face_rotation(he, m);

            const auto  p0_c      = m.point(source(he, m));
            const auto  p1_c      = m.point(target(he, m));
            const Vec3d p0        = internal::convert(p0_c);
            const Vec3d p1        = internal::convert(p1_c);
            const Vec3d delta     = position(mesh, to) - p0;
            const Vec3d delta_rot = rotation * delta;

            const Vec3d             p_star = p0 + delta_rot;
            const Kernel::Segment_3 segment(p0_c, p1_c);
            const Kernel::Ray_3     ray(
                    internal::point3_from_eigen(position(mesh, from)),
                    internal::vector3_from_eigen(p_star - position(mesh, from))
            );
            auto midpt = internal::edge_ray_intersection(segment, ray);

            if (!midpt.has_value()) {
                throw std::runtime_error(
                        "Expected ray intersection, but intersection not found!"
                );
            }

            return {position(mesh, from),
                    internal::convert(midpt.value()),
                    position(mesh, to)};
        }
    }

    Geodesic res = (*mesh.cgal()._geodesic_constructor)(from, to);

    assert(mdv::condition::are_equal(res.front(), position(mesh, from))
           && mdv::condition::are_equal(res.back(), position(mesh, to)));

    if (res.size() >= 2 && (res[0] - res[1]).norm() < 1e-12) res.erase(res.begin());
    const auto end = res.size() - 1;
    if (res.size() >= 2 && (res[end] - res[end - 1]).norm() < 1e-12)
        res.erase(res.end() - 1);


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

Eigen::MatrixXd
Mesh::get_vertex_matrix() const {
    Eigen::MatrixXd res(num_vertices(), 3);
    for (Index i = 0; i < num_vertices(); ++i)
        res.row(i) = position(*this, VertexId{i});
    return res;
}

Mesh::FaceMatrix
Mesh::get_face_matrix() const {
    FaceMatrix res(num_faces(), 3);
    for (Index i = 0; i < num_faces(); ++i) {
        const auto [b0, b1, b2] = vertices(*this, FaceId{i});
        res.row(i) << b0.value, b1.value, b2.value;
    }
    return res;
}

Eigen::MatrixXd
Mesh::get_face_matrix_double() const {
    const FaceMatrix mat = get_face_matrix();
    Eigen::MatrixXd  res(mat.rows(), mat.cols());
    for (long i = 0; i < mat.rows(); ++i) {
        for (long j = 0; j < mat.cols(); ++j) {
            res(i, j) = static_cast<double>(mat(i, j));
        }
    }
    return res;
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

    const auto   n_ref     = normal(mesh, face(mesh, pt));
    const double cos_angle = std::cos(max_normal_angle * M_PI / 180.0);

    const auto classify = [&cos_angle, &n_ref, &mesh](FaceId f) -> VisitState {
        const auto n = normal(mesh, f);
        if (n.dot(n_ref) > cos_angle) return VisitState::VALID_FACE;
        return VisitState::INVALID_FACE;
    };

    // Recursive lambda
    // https://stackoverflow.com/questions/78166176/how-can-i-write-an-inline-recursive-lambda-in-c
    const auto propagate = [&](const auto& self, const Index& id) -> void {
        if (face_class[id] != VisitState::UNVISITED) return;

        const auto f   = FaceId{id};
        face_class[id] = classify(f);
        if (face_class[id] == VisitState::INVALID_FACE) return;

        const auto [f_id1, f_id2, f_id3] = neighbours(mesh, f);
        self(self, f_id1.value);
        self(self, f_id2.value);
        self(self, f_id3.value);
    };

    mesh.logger().info(
            "Constructing submesh by from point {} - maximum normal angle: {}deg",
            describe(mesh, pt),
            max_normal_angle
    );
    propagate(propagate, face(mesh, pt).value);

    const auto& m_original = internal::cgal_mesh(mesh);
    auto        m_new      = m_original;

    long n_removed = 0;
    for (Index i = 0; i < mesh.num_faces(); ++i) {
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

}  // namespace mdv::mesh
