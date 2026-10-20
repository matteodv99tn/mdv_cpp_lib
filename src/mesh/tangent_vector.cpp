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

namespace mdv::mesh {

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
TangentVector::TangentVector(
        const Mesh&           mesh,
        Point                 app_point,
        Vec3d                 v,
        Type                  type,
        std::optional<FaceId> preferred
) :
        _pt(app_point), _vec(std::move(v)), _type(type) {
    using mdv::condition::are_orthogonal;

    if (!_vec.allFinite()) throw std::runtime_error("Nonfinite tangent vector");
    (void)position(mesh, _pt);
    _f = preferred.value_or(mesh::face(mesh, _pt));
    (void)internal::face_location(internal::cgal_mesh(mesh), _pt, _f);
    const auto point_type = location_type(_pt);

    if (point_type == LocationType::INSIDE_FACE) {
        validate_app_point_inside_face(mesh);
        return;
    }

    fill_halfedge(mesh);
    assert(_he.has_value());

    if (point_type == LocationType::ON_EDGE) {
        validate_app_point_on_edge(mesh);
        _f = mesh::face(mesh, *_he);
        return;
    }

    validate_app_point_on_vertex(mesh);


    // In case if the point is on a edge, you may switch the halfedge to ensure that
    // the vector is orthogonal to the face (which may be the opposite one)
    const auto* desc = _pt.get_as<PointOnEdge>();
    const bool  shall_switch =
            (desc != nullptr) && !are_orthogonal(normal(mesh, _f), _vec);
    if (shall_switch) _pt = flip(mesh, *desc);

    assert(mdv::condition::are_orthogonal(normal(mesh, _f), _vec));
}

void
TangentVector::validate_app_point_inside_face(const Mesh& mesh) {
    assert(location_type(_pt) == LocationType::INSIDE_FACE);
    if (_type == ALONG_EDGE) {
        throw std::runtime_error(
                "Cannot create tangent vector along edge when the application point is "
                "inside the face"
        );
    }
    if (!condition::are_orthogonal(normal(mesh, _f), _vec)) {
        throw std::runtime_error(
                "Provided vector is not orthogonal to the face normal"
        );
    }
}

void
TangentVector::validate_app_point_on_edge(const Mesh& mesh) {
    using namespace mdv::condition;
    assert(location_type(_pt) == LocationType::ON_EDGE);
    assert(_he.has_value());

    const auto switch_halfedge = [this, &mesh]() {
        _pt = flip(mesh, *_pt.get_as<PointOnEdge>());
        fill_halfedge(mesh);
    };


    if (_type == ALONG_EDGE) {
        const auto he_dir = direction(mesh, *_he);
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
    auto is_valid_halfedge = [this, &mesh](HalfEdgeId he) -> bool {
        return are_orthogonal(normal(mesh, mesh::face(mesh, he)), _vec)
               && inbound_direction(mesh, he).dot(_vec) > 0.0;
    };

    if (!is_valid_halfedge(_he.value())) switch_halfedge();

    if (!is_valid_halfedge(_he.value())) {
        throw std::runtime_error(
                "Unable to construct tangent vector on halfedge pointing inside a face"
        );
    }
}

void
TangentVector::validate_app_point_on_vertex(const Mesh& mesh) {
    using namespace condition;
    assert(location_type(_pt) == LocationType::ON_VERTEX);

    if (_type == ALONG_EDGE) {
        if (!are_parallel(direction(mesh, *_he).normalized(), _vec.normalized())) {
            throw std::runtime_error(
                    "Unable to construct tangent vector on vertex with vector along an "
                    "halfedge"
            );
        }
        return;
    }

    // If inside face
    const auto h  = *_he;
    const auto v1 = direction(mesh, h).normalized();
    const auto v3 = direction(mesh, twin(mesh, prev(mesh, h))).normalized();
    assert(mesh::face(mesh, h) == _f);

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
TangentVector::fill_halfedge(const Mesh& mesh) {
    using VertexIndex = internal::CgalImpl::CgalVertexIndex;
    using FaceIndex   = internal::CgalImpl::CgalFaceIndex;
    using HeIndex     = internal::CgalImpl::CgalHalfEdgeIndex;

    assert(location_type(_pt) != LocationType::INSIDE_FACE);

    const auto* he_descriptor = _pt.get_as<PointOnEdge>();
    if (he_descriptor != nullptr) {
        _he = he_descriptor->he;
        return;
    }

    const auto* v_descriptor = _pt.get_as<PointOnVertex>();
    _he                      = mesh::halfedge(mesh, v_descriptor->v, _f);
}

TangentVector
TangentVector::from_tip_position(
        const Mesh& mesh, const Point& origin, const CartesianPoint& tip
) {
    const Vec3d p0  = position(mesh, origin);
    const Vec3d vec = tip - p0;
    return from_ambient_vector(mesh, origin, vec);
}

TangentVector
TangentVector::from_ambient_vector(
        const Mesh& mesh, const Point& origin, const Eigen::Vector3d& vec
) {
    (void)position(mesh, origin);
    if (!vec.allFinite()) throw std::runtime_error("Nonfinite ambient vector");
    const auto loc_type = location_type(origin);

    if (loc_type == LocationType::INSIDE_FACE) {
        return {mesh,
                origin,
                normal_projection(vec, normal(mesh, mesh::face(mesh, origin))),
                INSIDE_FACE};
    }

    if (loc_type == LocationType::ON_EDGE) {
        return from_ambient_vector_on_edge(mesh, *origin.get_as<PointOnEdge>(), vec);
    }

    return from_ambient_vector_on_vertex(mesh, *origin.get_as<PointOnVertex>(), vec);
}

TangentVector
TangentVector::from_ambient_vector_on_edge(
        const Mesh& mesh, const PointOnEdge& origin, const Eigen::Vector3d& vec
) {
    using VertexIndex    = internal::CgalImpl::CgalVertexIndex;
    using HeIndex        = internal::CgalImpl::CgalHalfEdgeIndex;
    using FaceIndex      = internal::CgalImpl::CgalFaceIndex;
    using ProjectionInfo = std::tuple<Vec3d, Type, HeIndex>;

    const auto  he = origin.he;
    const auto& m  = internal::cgal_mesh(mesh);

    const auto project_on_halfedge = [&vec, &mesh](const HeIndex he_id) -> Vec3d {
        return project_along_direction(
                vec, direction(mesh, internal::from_cgal(he_id))
        );
    };
    const auto project_on_face = [&vec, &mesh, &m](const HeIndex he_id) -> Vec3d {
        const auto f = internal::from_cgal(CGAL::face(he_id, m));
        if (!f.is_valid()) return Vec3d::Zero();
        const auto proj = normal_projection(vec, normal(mesh, f));
        if (proj.dot(inbound_direction(mesh, internal::from_cgal(he_id))) <= 0.0)
            return Vec3d::Zero();
        return proj;
    };

    const auto    he_id      = internal::to_cgal(he);
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

    const auto point_constructor =
            [&origin, &he_id, &mesh](const HeIndex& actual_id) -> PointOnEdge {
        if (actual_id == he_id) return origin;
        return flip(mesh, origin);
    };

    return {
            mesh,
            point_constructor(std::get<2>(best_candidate)),
            std::get<0>(best_candidate),
            std::get<1>(best_candidate),
    };
}

TangentVector
TangentVector::from_ambient_vector_on_vertex(
        const Mesh&            mesh,
        const PointOnVertex&   application_point,
        const Eigen::Vector3d& vec
) {
    using VertexIndex = internal::CgalImpl::CgalVertexIndex;
    using HeIndex     = internal::CgalImpl::CgalHalfEdgeIndex;
    using FaceIndex   = internal::CgalImpl::CgalFaceIndex;

    using ProjectionInfo = std::tuple<Vec3d, Type, FaceIndex>;

    const auto& m    = internal::cgal_mesh(mesh);
    const auto  v_id = internal::to_cgal(application_point.v);

    const auto halfedges = CGAL::halfedges_around_source(v_id, m) | rs::to_vector;

    // Candidates inside faces
    const auto project_on_face = [&vec, &mesh](const HeIndex he_id) -> Vec3d {
        return normal_projection(
                vec, normal(mesh, mesh::face(mesh, internal::from_cgal(he_id)))
        );
    };

    const auto retrieve_vectors =
            [&vec, &m, &mesh](const HeIndex& he_id) -> std::array<Vec3d, 3> {
        const auto       opposite_he_id = opposite(prev(he_id, m), m);
        const HalfEdgeId he1            = internal::from_cgal(he_id);
        const HalfEdgeId he2            = internal::from_cgal(opposite_he_id);
        assert(mdv::condition::are_parallel(
                direction(mesh, he1).cross(direction(mesh, he2)),
                normal(mesh, mesh::face(mesh, he1))
        ));
        return {
                direction(mesh, he1),
                normal_projection(vec, normal(mesh, mesh::face(mesh, he1))),
                direction(mesh, he2),
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
                          project_on_face(he_id),
                          Type::INSIDE_FACE,
                          CGAL::face(he_id, m)
                  );
              })
            | rs::to<std::vector<ProjectionInfo>>;

    // Candidates inside faces
    const auto project_on_halfedge = [&vec, &mesh](const HeIndex he_id) -> Vec3d {
        const Vec3d proj = project_along_direction(
                vec, direction(mesh, internal::from_cgal(he_id))
        );
        if (proj.dot(vec) <= 0.0) return Vec3d::Zero();
        return proj;
    };
    const auto along_edge_candidates =
            halfedges | rv::transform([project_on_halfedge, &m](const HeIndex& he_id) {
                return std::make_tuple(
                        project_on_halfedge(he_id),
                        Type::ALONG_EDGE,
                        CGAL::face(he_id, m)
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

    return {
            mesh,
            application_point,
            std::get<0>(best_candidate),
            std::get<1>(best_candidate),
            internal::from_cgal(std::get<2>(best_candidate)),
    };
}

Eigen::Vector3d
tip(const Mesh& mesh, const TangentVector& v) {
    return position(mesh, v.application_point()) + v.cartesian_vector();
}

void
TangentVector::scale(double factor) {
    _vec *= factor;
}

void
TangentVector::normalise() {
    _vec.normalize();
}

TangentVector
TangentVector::normalised() const {
    TangentVector res(*this);
    res.normalise();
    return res;
}

namespace internal {

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

}  // namespace internal
}  // namespace mdv::mesh
