#include "mdv/mesh/tangent_vector.hpp"

#include <CGAL/Surface_mesh/Surface_mesh.h>

#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/all.hpp>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/utils/conditions.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::CartesianPoint;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;

namespace rs = ::ranges;
namespace rv = ::ranges::views;

namespace {
mdv::Vec3d
normal_projection(const mdv::Vec3d& vec, const mdv::Vec3d& normal) {
    assert(mdv::condition::is_unit_norm(normal));
    return (mdv::Mat3d::Identity() - normal * normal.transpose()) * vec;
}

mdv::Vec3d
project_along_direction(const mdv::Vec3d& vec, mdv::Vec3d dir) {
    dir.normalize();
    return dir * dir.dot(vec);
}

}  // namespace

// \endcond

//  _____                            _ __     __        _
// |_   _|_ _ _ __   __ _  ___ _ __ | |\ \   / /__  ___| |_ ___  _ __
//   | |/ _` | '_ \ / _` |/ _ \ '_ \| __\ \ / / _ \/ __| __/ _ \| '__|
//   | | (_| | | | | (_| |  __/ | | | |_ \ V /  __/ (__| || (_) | |
//   |_|\__,_|_| |_|\__, |\___|_| |_|\__| \_/ \___|\___|\__\___/|_|
//                  |___/
TangentVector::TangentVector(const Point& app_point, const Vec3d& v, Type type) :
        _pt(app_point), _vec(v), _type(type) {
    using mdv::condition::are_orthogonal;

    const auto point_type = location_type(app_point);

    if (point_type == LocationType::INSIDE_FACE) {
        validate_app_point_inside_face();
        return;
    }

    fill_halfedge();
    assert(_he.has_value());

    if (point_type == LocationType::ON_EDGE) {
        validate_app_point_on_edge();
        return;
    }

    validate_app_point_on_vertex();


    // In case if the point is on a edge, you may switch the halfedge to ensure that
    // the vector is orthogonal to the face (which may be the opposite one)
    const auto* desc = _pt.get_as<Point::PointOnEdgeDescriptor>();
    const bool  shall_switch =
            (desc != nullptr) && !are_orthogonal(_pt.face().normal(), _vec);
    if (shall_switch) _pt = desc->display_in_opposite_halfedge();

    assert(mdv::condition::are_orthogonal(_pt.face().normal(), _vec));
}

void
TangentVector::validate_app_point_inside_face() {
    assert(location_type(_pt) == LocationType::INSIDE_FACE);
    if (_type == ALONG_EDGE) {
        throw std::runtime_error(
                "Cannot create tangent vector along edge when the application point is "
                "inside the face"
        );
    }
    if (!condition::are_orthogonal(_pt.face().normal(), _vec)) {
        throw std::runtime_error(
                "Provided vector is not orthogonal to the face normal"
        );
    }
}

void
TangentVector::validate_app_point_on_edge() {
    using namespace mdv::condition;
    assert(location_type(_pt) == LocationType::ON_EDGE);
    assert(_he.has_value());

    const auto switch_halfedge = [this]() {
        _pt = _pt.get_as<Point::PointOnEdgeDescriptor>()
                      ->display_in_opposite_halfedge();
        fill_halfedge();
    };


    if (_type == ALONG_EDGE) {
        const auto he_dir = _he.value().direction();
        if (!are_parallel(he_dir, _vec)) {
            throw std::runtime_error(
                    "The provided vector is not parallel to the edge upon which it "
                    "shall be constructed!"
            );
        }
        if (he_dir.dot(_vec) < 0.0) switch_halfedge();
        return;
    }

    // If inside face
    auto is_valid_halfedge = [this](const HalfEdge& he) -> bool {
        return are_orthogonal(he.face().normal(), _vec)
               && he.inbound_direction().dot(_vec) > 0.0;
    };

    if (!is_valid_halfedge(_he.value())) switch_halfedge();

    if (!is_valid_halfedge(_he.value())) {
        throw std::runtime_error(
                "Unable to construct tangent vector on halfedge pointing inside a face"
        );
    }
}

void
TangentVector::validate_app_point_on_vertex() {
    using namespace condition;
    assert(location_type(_pt) == LocationType::ON_VERTEX);

    if (_type == ALONG_EDGE) {
        if (!are_parallel(_he->normalised_direction(), _vec.normalized())) {
            throw std::runtime_error(
                    "Unable to construct tangent vector on vertex with vector along an "
                    "halfedge"
            );
        }
        return;
    }

    // If inside face
    const auto v1 = _he->normalised_direction();
    const auto v3 = _he->prev().twin().normalised_direction();
    assert(_he->face().id() == _pt.face().id());

    if (!internal::are_coplanar(v1, _vec, v3)) {
        throw std::runtime_error(
                "Unable to construct tangent vector at vertex, as vector is not "
                "coplanar to the assigned face"
        );
    }
    if (!internal::vector_inside_triangle(v1, _vec, v3)) {
        throw std::runtime_error(
                "Unable to construct tangent vector at vertex, as vector is pointing "
                "outside the face"
        );
    }
}

void
TangentVector::fill_halfedge() {
    using VertexIndex = internal::CgalImpl::CgalVertexIndex;
    using FaceIndex   = internal::CgalImpl::CgalFaceIndex;
    using HeIndex     = internal::CgalImpl::CgalHalfEdgeIndex;

    assert(location_type(_pt) != LocationType::INSIDE_FACE);

    const auto* he_descriptor = _pt.get_as<Point::PointOnEdgeDescriptor>();
    if (he_descriptor != nullptr) {
        _he = he_descriptor->halfedge();
        return;
    }

    const auto* v_descriptor = _pt.get_as<Point::PointOnVertexDescriptor>();
    _he                      = HalfEdge{v_descriptor->vertex(), v_descriptor->face()};
}

TangentVector
TangentVector::from_tip_position(const Point& origin, const CartesianPoint& tip) {
    const Vec3d p0  = origin.position();
    const Vec3d vec = tip - p0;
    return from_ambient_vector(origin, vec);
}

TangentVector
TangentVector::from_ambient_vector(const Point& origin, const Eigen::Vector3d& vec) {
    const auto loc_type = location_type(origin);

    if (loc_type == LocationType::INSIDE_FACE)
        return {origin, normal_projection(vec, origin.face().normal()), INSIDE_FACE};

    if (loc_type == LocationType::ON_EDGE) {
        return from_ambient_vector_on_edge(
                *origin.get_as<Point::PointOnEdgeDescriptor>(), vec
        );
    }

    return from_ambient_vector_on_vertex(
            *origin.get_as<Point::PointOnVertexDescriptor>(), vec
    );
}

TangentVector
TangentVector::from_ambient_vector_on_edge(
        const Point::PointOnEdgeDescriptor& origin, const Eigen::Vector3d& vec
) {
    using VertexIndex    = internal::CgalImpl::CgalVertexIndex;
    using HeIndex        = internal::CgalImpl::CgalHalfEdgeIndex;
    using FaceIndex      = internal::CgalImpl::CgalFaceIndex;
    using ProjectionInfo = std::tuple<Vec3d, Type, HeIndex>;

    const auto& he   = origin.halfedge();
    const auto& mesh = he.mesh();
    const auto& m    = internal::get_mesh_impl(mesh);

    const auto project_on_halfedge = [&vec, &mesh](const HeIndex he_id) -> Vec3d {
        const HalfEdge he{mesh, he_id};
        return project_along_direction(vec, he.direction());
    };
    const auto project_on_face = [&vec, &mesh, &m](const HeIndex he_id) -> Vec3d {
        const Face     f{mesh, face(he_id, m)};
        const HalfEdge he{mesh, he_id};
        const auto     proj = normal_projection(vec, f.normal());
        if (proj.dot(he.inbound_direction()) <= 0.0) return Vec3d::Zero();
        return proj;
    };

    const HeIndex he_id{he.id()};
    const HeIndex twin_he_id = opposite(he_id, m);

    const std::vector<ProjectionInfo> candidates{
            {project_on_halfedge(he_id),      ALONG_EDGE,  he_id     },
            {project_on_halfedge(twin_he_id), ALONG_EDGE,  twin_he_id},
            {project_on_face(he_id),          INSIDE_FACE, he_id     },
            {project_on_face(twin_he_id),     INSIDE_FACE, twin_he_id},
    };

    const auto best_candidate =
            rs::max(candidates, std::less{}, [&vec](const ProjectionInfo& info) {
                return vec.dot(std::get<0>(info));
            });

    const auto point_constructor = [&origin, &he_id](
                                           const HeIndex& actual_id
                                   ) -> Point::PointOnEdgeDescriptor {
        if (actual_id == he_id) return origin;
        return origin.display_in_opposite_halfedge();
    };

    return {
            point_constructor(std::get<2>(best_candidate)),
            std::get<0>(best_candidate),
            std::get<1>(best_candidate),
    };
}

TangentVector
TangentVector::from_ambient_vector_on_vertex(
        const Point::PointOnVertexDescriptor& application_point,
        const Eigen::Vector3d&                vec
) {
    using VertexIndex = internal::CgalImpl::CgalVertexIndex;
    using HeIndex     = internal::CgalImpl::CgalHalfEdgeIndex;
    using FaceIndex   = internal::CgalImpl::CgalFaceIndex;

    using ProjectionInfo = std::tuple<Vec3d, Type, FaceIndex>;

    const auto&       v = application_point.vertex();
    const auto&       m = internal::get_mesh_impl(v);
    const VertexIndex v_id{v.id()};

    const auto halfedges = CGAL::halfedges_around_source(v_id, m) | rs::to_vector;

    // Candidates inside faces
    const auto project_on_face = [&vec, &v](const HeIndex he_id) -> Vec3d {
        return normal_projection(vec, HalfEdge{v.mesh(), he_id}.face().normal());
    };

    const auto retrieve_vectors =
            [&vec, &m, &v](const HeIndex& he_id) -> std::array<Vec3d, 3> {
        const auto     opposite_he_id = opposite(prev(he_id, m), m);
        const HalfEdge he1{v.mesh(), he_id};
        const HalfEdge he2{v.mesh(), opposite_he_id};
        assert(mdv::condition::are_parallel(
                he1.direction().cross(he2.direction()), he1.face().normal()
        ));
        return {
                he1.direction(),
                normal_projection(vec, he1.face().normal()),
                he2.direction(),
        };
    };
    const auto vector_inside_face = [](const auto& rng) -> bool {
        return internal::vector_inside_triangle(rng[0], rng[1], rng[2]);
    };
    const auto projected_vector_is_inside_face =
            [&retrieve_vectors, &vector_inside_face](const HeIndex& he_id) {
                return vector_inside_face(retrieve_vectors(he_id));
            };

    const auto inside_face_candidates =
            halfedges | rv::filter(projected_vector_is_inside_face)
            | rv::transform([project_on_face, &m](const HeIndex& he_id) {
                  return std::make_tuple(
                          project_on_face(he_id), Type::INSIDE_FACE, face(he_id, m)
                  );
              })
            | rs::to<std::vector<ProjectionInfo>>;

    // Candidates inside faces
    const auto project_on_halfedge = [&vec, &v](const HeIndex he_id) -> Vec3d {
        const HalfEdge he{v.mesh(), he_id};
        const Vec3d proj = project_along_direction(vec, he.direction());
        if (proj.dot(vec) <= 0.0) return Vec3d::Zero();
        return proj;
    };
    const auto along_edge_candidates =
            halfedges | rv::transform([project_on_halfedge, &m](const HeIndex& he_id) {
                return std::make_tuple(
                        project_on_halfedge(he_id), Type::ALONG_EDGE, face(he_id, m)
                );
            })
            | rs::to<std::vector<ProjectionInfo>>;

    // Find best fit
    const auto all_candidates =
            rv::concat(inside_face_candidates, along_edge_candidates);

    if (size(all_candidates) < 1) {
        throw std::runtime_error(
                "No valid candidates to initialise tangent vector on vertex"
        );
    }

    const auto best_candidate =
            rs::max(all_candidates, std::less{}, [&vec](const ProjectionInfo& info) {
                return vec.dot(std::get<0>(info));
            });

    Point::PointOnVertexDescriptor app_point{application_point};
    app_point.assign_face(Face{v.mesh(), std::get<2>(best_candidate)});
    return {
            app_point,
            std::get<0>(best_candidate),
            std::get<1>(best_candidate),
    };
}

TangentVector
TangentVector::unit_random(const Point& application_point) {
    const Vec3d n = application_point.face().normal();
    if (location_type(application_point) != LocationType::INSIDE_FACE) {
        throw std::runtime_error(
                "Don't know how to initialise unit random tangent vector on a point "
                "which is not inside the face"
        );
    }
    return {application_point,
            normal_projection(Vec3d::Random(), n).normalized(),
            INSIDE_FACE};
}

Eigen::Vector3d
TangentVector::tip() const noexcept {
    return application_point().position() + cartesian_vector();
}

mdv::Vec3d
TangentVector::cartesian_vector() const noexcept {
    return _vec;
}

void
TangentVector::scale(const double& factor) {
    _vec *= factor;
}

void
TangentVector::normalise() {
    _vec.normalize();
}

TangentVector
TangentVector::normalised() & {
    TangentVector res(*this);
    return {_pt, _vec.normalized(), type()};
}

TangentVector
TangentVector::normalised() && {
    const double len = _vec.norm();
    scale(1.0 / len);
    return *this;
}

namespace mdv::mesh::internal {

bool
vector_inside_triangle(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3) {
    if (v1.squaredNorm() < 1e-12 || v2.squaredNorm() < 1e-12
        || v3.squaredNorm() < 1e-12)
        return false;

    const Vec3d c1 = v1.cross(v2);
    const Vec3d c2 = v2.cross(v3);
    const Vec3d c3 = v1.cross(v3);

    if (c1.squaredNorm() < 1e-12 || c2.squaredNorm() < 1e-12
        || c3.squaredNorm() < 1e-12)
        return false;

    if (!are_coplanar(v1, v2, v3))
        throw std::runtime_error("Cannot compute order of non-coplanar vector");

    return c1.dot(c2) > 0.0 && c1.dot(c3) > 0.0;
}

bool
are_coplanar(const Vec3d& v1, const Vec3d& v2, const Vec3d& v3) {
    const Vec3d c1 = v1.cross(v2);
    const Vec3d c2 = v2.cross(v3);
    return mdv::condition::are_parallel(c1, c2);
}

}  // namespace mdv::mesh::internal
