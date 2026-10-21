#include <array>
#include <cassert>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cmath>
#include <cstdint>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>

#include "mdv/eigen_defines.hpp"
#include "mdv/macros.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/eigen_solvers.hpp"

namespace mdv::mesh {

namespace {
    using Kernel           = internal::CgalImpl::Kernel;
    using FaceIndex        = internal::CgalImpl::CgalFaceIndex;
    using VertexIndex      = internal::CgalImpl::CgalVertexIndex;
    using HalfEdgeIndex    = internal::CgalImpl::CgalHalfEdgeIndex;
    using VertexDescriptor = PointOnVertex;
    using CgalHalfEdge     = internal::CgalImpl::CgalHalfEdgeIndex;
    using CgalMesh         = internal::CgalImpl::Mesh;
    using Point3           = internal::CgalImpl::Point3;
    using Vec3             = internal::CgalImpl::Vec3;

    // Strictly inside f.
    // Corners: source(h), target(h), target(next(h)), h = halfedge(f).
    // db: barycentric velocity per unit arc length (sum(db) == 0).
    struct InFace {
        FaceIndex f;   // face
        Vec3d     b;   // barycentric point description
        Vec3d     db;  // vector direction in barycentric coordinates
    };

    // Direction originating at a halfedge interior and pointing inside face
    struct OnEdge {
        HalfEdgeIndex he;       // face(he) contains the vector
        double        t = 0.0;  // coordinate in (0,1) on the he
        Vec2d         d;        // direction in he coordinate system
                                // d[0] along he(source -> target); d[1] > 0 he-normal
    };

    // Direction along he moving towards target(he)
    // If t == 0, then the origin is the vertex
    struct AlongEdge {
        HalfEdgeIndex he;       // handles position and vector direction
        double        t = 0.0;  // coordinate in [0,1) on the he
    };

    // Direction is a rotation of "theta" from he origintating at source(he)
    struct OnVertex {
        HalfEdgeIndex he;
        double        theta = 0.0;
    };

    using ContinuationState = std::variant<InFace, OnEdge, AlongEdge, OnVertex>;

    // Functor for the variant that for each input continuation state outputs the next
    // continuation step and the corresponding step length
    struct ContinuationStepper {
        using Result = std::pair<ContinuationState, double>;

        MDV_NODISCARD Result
        operator()(const InFace& state) const {
            // Validate precondition
            assert(std::abs(state.b.sum() - 1.0) < 1e-12);
            assert(std::abs(state.db.sum()) < 1e-12);

            constexpr double eps_t = 1e-9;  // edge-parameter snapping to vertices

            const auto edge_vec = [this](const HalfEdgeIndex he) -> Vec3d {
                return internal::convert(
                        m.point(target(he, m)) - m.point(source(he, m))
                );
            };

            // Exit through the edge opposite corner k, the first coordinate to reach 0
            Eigen::Index k    = -1;
            double       dist = std::numeric_limits<double>::infinity();
            for (Eigen::Index i = 0; i < 3; ++i) {
                if (state.db(i) >= 0.0) continue;
                const double s = -state.b(i) / state.db(i);
                if (s < dist) {
                    dist = s;
                    k    = i;
                }
            }
            assert(k >= 0 && k < 3);
            if (k < 0) {
                throw std::runtime_error(
                        fmt::format(
                                "exponential_map: no exit found from face {}",
                                static_cast<std::size_t>(state.f)
                        )
                );
            }

            Vec3d bexit    = state.b + dist * state.db;  // bary coords at crossing
            bexit(k)       = 0.0;
            const double t = bexit((k + 2) % 3);  // coordinate along crossing he

            const HalfEdgeIndex h = halfedge(state.f, m);
            const std::array    opposite_he{next(h, m), prev(h, m), h};
            const HalfEdgeIndex he = opposite_he[static_cast<std::size_t>(k)];

            // 3D unit direction: u = db0*P0 + db1*P1 + db2*P2, with sum(db) == 0
            const Vec3d u =
                    state.db(1) * edge_vec(h) - state.db(2) * edge_vec(prev(h, m));

            if ((t < eps_t) || (t > 1.0 - eps_t)) {
                // Vertex hit: straightest geodesic, leave at half the total angle.
                // h_ref is the halfedge of the face whose source is the hit vertex.
                const HalfEdgeIndex h_ref = (t < eps_t) ? he : next(he, m);
                const Vec3d         e     = edge_vec(h_ref);
                const Vec3d         back  = -u;
                const double beta = std::atan2(e.cross(back).norm(), e.dot(back));

                // Total angle around source(h_ref), walking c <- opposite(prev(c))
                double        angle = 0.0;
                HalfEdgeIndex c     = h_ref;
                do {
                    if (CGAL::is_border(c, m)) {
                        throw std::runtime_error(
                                fmt::format(
                                        "exponential_map: border reached at vertex {}",
                                        static_cast<std::size_t>(source(h_ref, m))
                                )
                        );
                    }
                    const Vec3d e1 = edge_vec(c);
                    const Vec3d e2 = -edge_vec(prev(c, m));
                    angle += std::atan2(e1.cross(e2).norm(), e1.dot(e2));
                    c = opposite(prev(c, m), m);
                } while (c != h_ref);

                const double theta = std::fmod(beta + (0.5 * angle), angle);
                return {
                        OnVertex{.he = h_ref, .theta = theta},
                        dist
                };
            }

            // Edge crossing: express u in the frame of he, then flip into opposite(he)
            const HalfEdgeIndex twin = opposite(he, m);
            if (CGAL::is_border(twin, m)) {
                throw std::runtime_error(
                        fmt::format(
                                "exponential_map: border reached at halfedge {}",
                                static_cast<std::size_t>(he)
                        )
                );
            }
            const Vec3d x = edge_vec(he).normalized();
            const Vec3d n = edge_vec(he).cross(edge_vec(next(he, m))).normalized();
            const Vec3d y = n.cross(x);  // in-plane, pointing into face(he)
            const Vec2d d = Vec2d(u.dot(x), u.dot(y)).normalized();
            assert(d.y() < 0.0);  // outbound from face(he)
            return {
                    OnEdge{.he = twin, .t = 1.0 - t, .d = -d},
                    dist
            };
        }

        MDV_NODISCARD Result
        operator()(const OnEdge& state) {}

        MDV_NODISCARD Result
        operator()(const AlongEdge& state) {}

        MDV_NODISCARD Result
        operator()(const OnVertex& state) {}

        const CgalMesh& m;
    };

    CgalHalfEdge
    get_halfedge(
            const FaceIndex& f_id, const Kernel::Point_3& pt, const CgalMesh& mesh
    ) {
        for (const auto he : CGAL::halfedges_around_face(halfedge(f_id, mesh), mesh)) {
            const Kernel::Segment_3 seg(
                    mesh.point(source(he, mesh)), mesh.point(target(he, mesh))
            );
            if (CGAL::squared_distance(seg, pt) < 1e-12) return he;
        }
        throw std::runtime_error(
                "Mesh::exponential_map: unable to find proper halfedge"
        );
    }

    enum IntersectionType : std::uint8_t {
        TRIANGLE_VERTEX_INTERSECTION,
        TRIANGLE_EDGE_INTERSECTION
    };

    using IntersectionResult = std::pair<Kernel::Point_3, IntersectionType>;

    struct VectorInsideFace {};

    struct VectorAlongEdge {};

    IntersectionResult
    compute_intersection(
            const Kernel::Ray_3      ray,
            const Kernel::Triangle_3 tri,
            VectorInsideFace /* unused */
    ) {
        const auto& v0 = tri.vertex(0);
        const auto& v1 = tri.vertex(1);
        const auto& v2 = tri.vertex(2);

        const Kernel::Segment_3 edge0(v0, v1);
        const Kernel::Segment_3 edge1(v0, v2);
        const Kernel::Segment_3 edge2(v1, v2);
        const auto              i0 = internal::edge_ray_intersection(edge0, ray);
        const auto              i1 = internal::edge_ray_intersection(edge1, ray);
        const auto              i2 = internal::edge_ray_intersection(edge2, ray);

        std::size_t sols_found = 0;
        if (i0.has_value()) ++sols_found;
        if (i1.has_value()) ++sols_found;
        if (i2.has_value()) ++sols_found;

        Kernel::Point_3 res;

        if (sols_found == 2) {
            if (i0.has_value()) res = i0.value();
            else res = i1.value();
            return std::make_pair(res, TRIANGLE_VERTEX_INTERSECTION);
        }

        if (i0.has_value()) {
            res = i0.value();
            assert(CGAL::squared_distance(edge0, res) < 1e-12);
        }
        if (i1.has_value()) {
            res = i1.value();
            assert(CGAL::squared_distance(edge1, res) < 1e-12);
        }
        if (i2.has_value()) {
            res = i2.value();
            assert(CGAL::squared_distance(edge2, res) < 1e-12);
        }
        return std::make_pair(res, TRIANGLE_EDGE_INTERSECTION);
    }

    /*
     * Retrieves the halfedge on face "f" which is "most parallel" to the vector "v"
     */
    HalfEdgeIndex
    get_most_parallel_halfedge(
            const Kernel::Vector_3          v,
            const FaceIndex                 f,
            const internal::CgalImpl::Mesh& m
    ) {
        double        best_dot = 0.0;
        HalfEdgeIndex closest_he{invalid_index};
        for (const auto he : CGAL::halfedges_around_face(halfedge(f, m), m)) {
            const Kernel::Vector_3 dir =
                    m.point(source(he, m)) - m.point(target(he, m));
            const double len      = std::sqrt(dir.squared_length());
            const double this_dot = std::abs(dir * v) / len;
            if (this_dot > best_dot) {
                best_dot   = this_dot;
                closest_he = he;
            }
        }
        assert(closest_he != HalfEdgeIndex{invalid_index});
        return closest_he;
    }

    IntersectionResult
    compute_intersection(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const Kernel::Vector_3          v,
            const internal::CgalImpl::Mesh& m,
            VectorAlongEdge /* unused */
    ) {
        const auto he = get_most_parallel_halfedge(v, f, m);
        const auto v0 = m.point(source(he, m));
        const auto v1 = m.point(target(he, m));
        const auto d0 = v0 - p;
        const auto d1 = v1 - p;

        assert((d0 * v) * (d1 * v) <= 0.0);

        if (d0 * v > 1e-18) return std::make_pair(v0, TRIANGLE_VERTEX_INTERSECTION);
        return std::make_pair(v1, TRIANGLE_VERTEX_INTERSECTION);
    }

    /**
     * @brief Given a vector "v" applied in point "p" on face "f", returns the
     * intersection of the ray given by the vector itself with the face.
     *
     * The "type" is used to dispatch proper intersection calculation on wether the
     * tangent vector lies inside the face, or along an edge of the face.
     */
    IntersectionResult
    compute_intersection(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const Kernel::Vector_3          v,
            const TangentVector::Type       type,
            const internal::CgalImpl::Mesh& m
    ) {
        if (type == TangentVector::Type::INSIDE_FACE) [[likely]] {
            const Kernel::Ray_3                     ray(p, v);
            CGAL::Triangle_from_face_descriptor_map tri_gen(&m);
            const Kernel::Triangle_3                tri = get(tri_gen, f);
            return compute_intersection(ray, tri, VectorInsideFace{});
        }
        return compute_intersection(p, f, v, m, VectorAlongEdge{});
    }

    /**
     * @brief Given a point "p" which is assumed to be a vertex for the face "f",
     * retrieves the vertex id on the mesh "m" and the halfedge on "f" that has "v" as
     * source.
     */
    std::pair<VertexIndex, HalfEdgeIndex>
    get_vertex_halfedge_pair(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const internal::CgalImpl::Mesh& m
    ) {
        for (const auto he : CGAL::halfedges_around_face(CGAL::halfedge(f, m), m)) {
            const auto v_id = source(he, m);
            if ((m.point(v_id) - p).squared_length() < 1e-18)
                return std::make_pair(v_id, he);
        }
        throw std::runtime_error("Provided point is not a vertex for the face");
    }

    struct PropagationResult {
        Kernel::Vector_3    v;
        FaceIndex           f;
        TangentVector::Type type = TangentVector::INSIDE_FACE;
    };

    /*
     * Given a vector "v" applied at point "p" on the edge of face "f" whose direction
     * is outbound the face, gives the projected inbound vector on the contiguous face.
     */
    PropagationResult
    propagate_along_edge(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const Kernel::Vector_3          v,
            const internal::CgalImpl::Mesh& m
    ) {
        using CGAL::Polygon_mesh_processing::compute_face_normal;
        const auto      he     = get_halfedge(f, p, m);
        const FaceIndex next_f = m.face(CGAL::opposite(he, m));

        if (next_f == FaceIndex{invalid_index})
            throw std::runtime_error("Boarder reached when computing exponential map");

        const auto& v0 = m.point(source(he, m));
        const auto& v1 = m.point(target(he, m));
        const Vec3d ax = internal::convert(v1 - v0).normalized();
        const Vec3d n1 = internal::convert(compute_face_normal(f, m));
        const Vec3d n2 = internal::convert(compute_face_normal(next_f, m));

        // Binormal axis computation
        const Vec3d b1 = -ax.cross(n1);
        const Vec3d b2 = ax.cross(n2);

        // Compute rotation
        const Eigen::Quaterniond q            = quaternion_from_two_vectors(-b1, b2);
        const Vec3d              v_next_eigen = q * internal::convert(v);
        const auto               v_next = internal::vector3_from_eigen(v_next_eigen);

        using mdv::condition::are_orthogonal, mdv::condition::is_unit_norm;
        assert(are_orthogonal(b1, ax) && are_orthogonal(b1, n1) && is_unit_norm(b1));
        assert(are_orthogonal(b2, ax) && are_orthogonal(b2, n2) && is_unit_norm(b2));
        assert(are_orthogonal(v_next_eigen, n2));

        return {.v = v_next, .f = next_f, .type = TangentVector::INSIDE_FACE};
    }

    /*
     * Given a vector "v" applied at point "p" which is a vertex of face "f" whose
     * direction is outbound the face, gives the projected inbound vector on the
     * contiguous face.
     */
    PropagationResult
    propagate_along_vertex(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const Kernel::Vector_3          v,
            const internal::CgalImpl::Mesh& m
    ) {
        using HalfEdgeCirculator =
                CGAL::Halfedge_around_source_circulator<internal::CgalImpl::Mesh>;
        using CGAL::Polygon_mesh_processing::compute_face_normal;
        using internal::convert, internal::total_curvature_deg,
                internal::vector_inside_triangle, internal::vector3_from_eigen;
        using mdv::condition::are_orthogonal;

        const auto he_to_vec3 = [m](const HalfEdgeIndex& he_id) -> Kernel::Vector_3 {
            return m.point(target(he_id, m)) - m.point(source(he_id, m));
        };
        const auto length = [](const Kernel::Vector_3& v) {
            return CGAL::approximate_sqrt(v.squared_length());
        };
        const auto normalize = [length](const Kernel::Vector_3& v) -> Kernel::Vector_3 {
            return v / length(v);
        };

        // Note: CGAL approximate angle works in degrees
        const auto [v_id, start_he] = get_vertex_halfedge_pair(p, f, m);
        const double tot_curv       = total_curvature_deg(m, v_id);
        const double target_angle   = tot_curv * 0.5;
        double traversed_angle      = CGAL::approximate_angle(-v, he_to_vec3(start_he));

        assert(face(start_he, m) == f);

        HalfEdgeCirculator he_circ(start_he, m);
        while (traversed_angle < target_angle - 1e-9) {
            const auto curr_he = *he_circ;
            ++he_circ;
            const auto next_he = *he_circ;
            traversed_angle +=
                    CGAL::approximate_angle(he_to_vec3(curr_he), he_to_vec3(next_he));
        }

        const auto   next_f       = face(*he_circ, m);
        const double excess_angle = traversed_angle - target_angle;
        const auto   n            = convert(compute_face_normal(next_f, m));
        const auto   d            = convert(normalize(he_to_vec3(*he_circ)));

        if (mdv::condition::is_zero(excess_angle)) {
            return {
                    .v    = normalize(he_to_vec3(*he_circ)) * length(v),
                    .f    = next_f,
                    .type = TangentVector::ALONG_EDGE,
            };
        }

        const Eigen::AngleAxisd rot(excess_angle * M_PI / 180.0, n);
        const Vec3d             vec_dir = rot * d;

        assert(are_orthogonal(n, d));
        assert(are_orthogonal(vec_dir, n));
        assert(vector_inside_triangle(
                convert(he_to_vec3(*he_circ)),
                vec_dir,
                convert(he_to_vec3(*std::prev(he_circ)))
        ));
        return {
                .v    = vector3_from_eigen(vec_dir * length(v)),
                .f    = next_f,
                .type = TangentVector::INSIDE_FACE,
        };
    }

    PropagationResult
    propagate_vector(
            const IntersectionType          intersection_type,
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const Kernel::Vector_3          v,
            const internal::CgalImpl::Mesh& m
    ) {
        if (intersection_type == TRIANGLE_EDGE_INTERSECTION)
            return propagate_along_edge(p, f, v, m);
        return propagate_along_vertex(p, f, v, m);
    }

    std::pair<Kernel::Point_3, FaceIndex>
    exponential_map_impl(
            const Kernel::Point_3           p0,
            const FaceIndex                 f_id,
            const Kernel::Vector_3          vec,
            const TangentVector::Type       vec_type,
            const internal::CgalImpl::Mesh& mesh,
            Geodesic*                       geod
    ) {
        if (geod != nullptr) geod->emplace_back(internal::convert(p0));

        const auto [pstar, pstar_type] =
                compute_intersection(p0, f_id, vec, vec_type, mesh);

        const double dist_p0_pstar = CGAL::squared_distance(p0, pstar);
        const double vec_len       = vec.squared_length();

        // Check if the vector is within the face
        if (vec_len < dist_p0_pstar) {
            const Kernel::Point_3 res = p0 + vec;
            return std::make_pair(res, f_id);
        }

        const Kernel::Vector_3 v_cut  = pstar - p0;
        const Kernel::Vector_3 v_left = vec - v_cut;
        assert(v_left.squared_length() < vec.squared_length());

        const auto [v_next, next_face_id, next_type] =
                propagate_vector(pstar_type, pstar, f_id, v_left, mesh);

        // TODO: check that the updated vector points "internally" to the face
        return exponential_map_impl(pstar, next_face_id, v_next, next_type, mesh, geod);
    }
}  // namespace

Point
exponential_map(const Mesh& mesh, const TangentVector& v, Geodesic* geod) {
    if (mdv::condition::is_zero_norm(v.cartesian_vector()))
        return v.application_point();

    const auto  p0 = internal::point3_from_eigen(position(mesh, v.application_point()));
    const auto  f_id       = internal::to_cgal(v.face());
    const auto& m          = internal::cgal_mesh(mesh);
    const auto  vec        = internal::vector3_from_eigen(v.cartesian_vector());
    const auto [pfinal, f] = exponential_map_impl(p0, f_id, vec, v.type(), m, geod);
    const auto res         = from_cartesian(mesh, internal::convert(pfinal));
    if (geod != nullptr) geod->emplace_back(position(mesh, res));
    return res;
}
}  // namespace mdv::mesh
