#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <fmt/format.h>
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

namespace mdv::mesh {

namespace {
    using FaceIndex     = internal::CgalImpl::CgalFaceIndex;
    using HalfEdgeIndex = internal::CgalImpl::CgalHalfEdgeIndex;
    using CgalMesh      = internal::CgalImpl::Mesh;

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

    // Outcome of a continuation: the next state and the distance travelled to reach
    // it. Steps travelling inside a face also store the followed ray
    // b(s) = b + s*db in face f, so that the walk can stop before reaching "next".
    struct Step {
        ContinuationState next;
        double            dist = 0.0;
        FaceIndex         f;
        Vec3d             b  = Vec3d::Zero();
        Vec3d             db = Vec3d::Zero();
    };

    // Functor for the variant that for each input continuation state outputs the next
    // continuation step and the corresponding step length
    struct ContinuationStepper {
        MDV_NODISCARD Step
        operator()(const InFace& state) const {
            assert(std::abs(state.b.sum() - 1.0) < 1e-12);
            assert(std::abs(state.db.sum()) < 1e-12);
            return ray_step(state.f, state.b, state.db);
        }

        MDV_NODISCARD Step
        operator()(const OnEdge& state) const {
            if (CGAL::is_border(state.he, m)) {
                throw std::runtime_error(
                        fmt::format(
                                "exponential_map: border reached at halfedge {}",
                                static_cast<std::size_t>(opposite(state.he, m))
                        )
                );
            }
            assert(state.d.y() > 0.0);  // inbound

            const auto j   = corner_index(state.he);
            Vec3d      b   = Vec3d::Zero();
            b(j)           = 1.0 - state.t;
            b((j + 1) % 3) = state.t;
            return ray_step(face(state.he, m), b, bary_velocity(state.he, state.d));
        }

        MDV_NODISCARD Step
        operator()(const AlongEdge& state) const {
            assert(state.t >= 0.0 && state.t < 1.0);

            // At target(he) the backward direction is opposite(he) (fan angle 0):
            // the straightest continuation leaves at half the total angle
            const double        dist = (1.0 - state.t) * edge_vec(state.he).norm();
            const HalfEdgeIndex back = opposite(state.he, m);
            return {
                    .next = OnVertex{.he = back, .theta = 0.5 * total_angle(back)},
                    .dist = dist
            };
        }

        MDV_NODISCARD Step
        operator()(const OnVertex& state) const {
            if (std::isnan(state.theta)) {
                throw std::runtime_error(
                        fmt::format(
                                "exponential_map: border reached at vertex {}",
                                static_cast<std::size_t>(source(state.he, m))
                        )
                );
            }
            assert(state.theta >= 0.0);
            constexpr double eps_ang = 1e-9;  // angular snapping to fan edges

            // Fan walk from he until the corner c containing theta is found
            HalfEdgeIndex c     = state.he;
            double        alpha = state.theta;  // residual angle from c
            do {                                // NOLINT do while loop
                if (CGAL::is_border(c, m)) {
                    throw std::runtime_error(
                            fmt::format(
                                    "exponential_map: border reached at vertex {}",
                                    static_cast<std::size_t>(source(c, m))
                            )
                    );
                }
                const double        ca     = corner_angle(c);
                const HalfEdgeIndex c_next = opposite(prev(c, m), m);
                if (alpha <= ca) {
                    if (alpha < eps_ang) {
                        return {
                                .next = AlongEdge{.he = c, .t = 0.0}
                        };
                    }
                    if (alpha > ca - eps_ang) {
                        return {
                                .next = AlongEdge{.he = c_next, .t = 0.0}
                        };
                    }

                    // Ray from corner source(c) inside face(c): the only exit is
                    // next(c), since the other two coordinates are 0 with db > 0
                    Vec3d b            = Vec3d::Zero();
                    b(corner_index(c)) = 1.0;
                    const Vec2d d(std::cos(alpha), std::sin(alpha));
                    return ray_step(face(c, m), b, bary_velocity(c, d));
                }
                alpha -= ca;
                c = c_next;
            } while (c != state.he);

            // theta >= total angle by rounding only: same as theta = 0
            return {
                    .next = AlongEdge{.he = state.he, .t = 0.0}
            };
        }

        /*
         * Initial state from the tangent vector (unit direction, the length is
         * tracked by the caller). OnEdge states built here may lie within eps_t of a
         * vertex: the OnEdge continuation does not rely on that invariant, which is
         * only needed to avoid confusing the entry edge after a crossing.
         */
        MDV_NODISCARD ContinuationState
        initial_state(const TangentVector& v) const {
            const Point& p   = v.application_point();
            const Vec3d& vec = v.cartesian_vector();

            if (const auto* pf = p.get_as<PointInFace>()) {
                const FaceIndex f = internal::to_cgal(pf->f);
                return InFace{
                        .f = f, .b = Vec3d(pf->coords()), .db = bary_velocity(f, vec)
                };
            }

            if (const auto* pe = p.get_as<PointOnEdge>()) {
                const HalfEdgeIndex he = internal::to_cgal(pe->he);
                if (v.type() == TangentVector::ALONG_EDGE) {
                    assert(edge_vec(he).dot(vec) > 0.0);
                    return AlongEdge{.he = he, .t = pe->t};
                }
                assert(face(he, m) == internal::to_cgal(v.face()));
                const auto [x, y] = frame(he);
                const Vec2d d     = Vec2d(vec.dot(x), vec.dot(y)).normalized();
                return OnEdge{.he = he, .t = pe->t, .d = d};
            }

            // Application point on vertex: halfedge() is outgoing in face()
            assert(v.halfedge().has_value());
            const HalfEdgeIndex he = internal::to_cgal(*v.halfedge());
            if (v.type() == TangentVector::ALONG_EDGE) {
                // Only parallelism is validated: pick the outgoing edge of the corner
                // pointing along the vector
                if (edge_vec(he).dot(vec) > 0.0) return AlongEdge{.he = he, .t = 0.0};
                const HalfEdgeIndex other = opposite(prev(he, m), m);
                if (edge_vec(other).dot(vec) > 0.0)
                    return AlongEdge{.he = other, .t = 0.0};
                throw std::runtime_error(
                        fmt::format(
                                "exponential_map: vector at vertex {} does not point "
                                "along an outgoing edge",
                                static_cast<std::size_t>(source(he, m))
                        )
                );
            }
            const Vec3d  e     = edge_vec(he);
            const double theta = std::atan2(e.cross(vec).norm(), e.dot(vec));
            return OnVertex{.he = he, .theta = theta};
        }

        // Exact location of a state (the walk never stops on an InFace "next")
        MDV_NODISCARD Point
        location(const ContinuationState& state) const {
            if (const auto* s = std::get_if<OnEdge>(&state))
                return edge_point(s->he, s->t);
            if (const auto* s = std::get_if<AlongEdge>(&state)) {
                if (s->t == 0.0)
                    return PointOnVertex{internal::from_cgal(source(s->he, m))};
                return edge_point(s->he, s->t);
            }
            if (const auto* s = std::get_if<OnVertex>(&state))
                return PointOnVertex{internal::from_cgal(source(s->he, m))};
            const auto& s = std::get<InFace>(state);
            return PointInFace{
                    .f = internal::from_cgal(s.f), .bary = {s.b(0), s.b(1), s.b(2)}
            };
        }

        // Point at parameter t on he, expressed on the non-border side
        MDV_NODISCARD PointOnEdge
        edge_point(const HalfEdgeIndex he, const double t) const {
            if (CGAL::is_border(he, m))
                return {.he = internal::from_cgal(opposite(he, m)), .t = 1.0 - t};
            return {.he = internal::from_cgal(he), .t = t};
        }

        MDV_NODISCARD Vec3d
        edge_vec(const HalfEdgeIndex he) const {
            return internal::convert(m.point(target(he, m)) - m.point(source(he, m)));
        }

        const CgalMesh& m;

    private:
        static constexpr double eps_t = 1e-9;  // edge-parameter snapping to vertices

        // Index of corner source(he) in face(he): corners are source(h), target(h),
        // target(next(h)) with h = halfedge(face(he))
        MDV_NODISCARD Eigen::Index
                      corner_index(const HalfEdgeIndex he) const {
            const HalfEdgeIndex h = halfedge(face(he, m), m);
            if (he == h) return 0;
            if (he == next(h, m)) return 1;
            assert(he == prev(h, m));
            return 2;
        }

        // Frame of he: x along he (source -> target), y in-plane pointing into face(he)
        MDV_NODISCARD std::pair<Vec3d, Vec3d>
                      frame(const HalfEdgeIndex he) const {
            const Vec3d x = edge_vec(he).normalized();
            const Vec3d n = edge_vec(he).cross(edge_vec(next(he, m))).normalized();
            return {x, n.cross(x)};
        }

        /*
         * Barycentric velocity in face f of the direction u, projected on the face
         * plane and normalised (unit speed).
         * db_i = u . grad(b_i), with grad(b_i) = n x (edge opposite corner i) / (2A).
         */
        MDV_NODISCARD Vec3d
        bary_velocity(const FaceIndex f, const Vec3d& u) const {
            const HalfEdgeIndex h = halfedge(f, m);
            const std::array    opposite_he{next(h, m), prev(h, m), h};
            const Vec3d         n = edge_vec(h).cross(edge_vec(next(h, m)));

            const Vec3d n_hat = n.normalized();
            const Vec3d u_hat = (u - (n_hat * n_hat.dot(u))).normalized();

            Vec3d db;
            for (std::size_t i = 0; i < 3; ++i) {
                db(static_cast<Eigen::Index>(i)) =
                        u_hat.dot(n.cross(edge_vec(opposite_he.at(i))))
                        / n.squaredNorm();
            }
            return db;
        }

        // Barycentric velocity in face(he) of the unit direction d in the frame of he
        MDV_NODISCARD Vec3d
        bary_velocity(const HalfEdgeIndex he, const Vec2d& d) const {
            const auto [x, y] = frame(he);
            return bary_velocity(face(he, m), (d.x() * x) + (d.y() * y));
        }

        // Angle at source(c) inside face(c)
        MDV_NODISCARD double
        corner_angle(const HalfEdgeIndex c) const {
            const Vec3d e1 = edge_vec(c);
            const Vec3d e2 = -edge_vec(prev(c, m));
            return std::atan2(e1.cross(e2).norm(), e1.dot(e2));
        }

        // Total angle around source(he), walking the fan c <- opposite(prev(c)).
        // NaN on border vertices (open fan): the OnVertex continuation throws, so the
        // walk may still stop exactly at a border vertex.
        MDV_NODISCARD double
        total_angle(const HalfEdgeIndex he) const {
            double        angle = 0.0;
            HalfEdgeIndex c     = he;
            do {  // NOLINT do while loop
                if (CGAL::is_border(c, m))
                    return std::numeric_limits<double>::quiet_NaN();
                angle += corner_angle(c);
                c = opposite(prev(c, m), m);
            } while (c != he);
            return angle;
        }

        /*
         * Ray b(s) = b + s*db in face f (corners: source(h), target(h),
         * target(next(h)), h = halfedge(f)). Finds the first exit and classifies it
         * as a vertex hit or an edge crossing. Coordinates with db_i >= 0 (e.g. the
         * entry edge) are never candidates.
         */
        MDV_NODISCARD Step
        ray_step(const FaceIndex f, const Vec3d& b, const Vec3d& db) const {
            // Exit through the edge opposite corner k, the first coordinate to reach 0
            Eigen::Index k    = -1;
            double       dist = std::numeric_limits<double>::infinity();
            for (Eigen::Index i = 0; i < 3; ++i) {
                if (db(i) >= 0.0) continue;
                const double s = -b(i) / db(i);
                if (s < dist) {
                    dist = s;
                    k    = i;
                }
            }
            if (k < 0) {
                throw std::runtime_error(
                        fmt::format(
                                "exponential_map: no exit found from face {}",
                                static_cast<std::size_t>(f)
                        )
                );
            }

            Vec3d bexit    = b + dist * db;  // bary coords at crossing
            bexit(k)       = 0.0;
            const double t = bexit((k + 2) % 3);  // coordinate along crossing he

            const HalfEdgeIndex h = halfedge(f, m);
            const std::array    opposite_he{next(h, m), prev(h, m), h};
            const HalfEdgeIndex he = opposite_he[static_cast<std::size_t>(k)];

            // 3D unit direction: u = db0*P0 + db1*P1 + db2*P2, with sum(db) == 0
            const Vec3d u = db(1) * edge_vec(h) - db(2) * edge_vec(prev(h, m));

            if ((t < eps_t) || (t > 1.0 - eps_t)) {
                // Vertex hit: straightest geodesic, leave at half the total angle.
                // h_ref is the halfedge of the face whose source is the hit vertex.
                const HalfEdgeIndex h_ref = (t < eps_t) ? he : next(he, m);
                const Vec3d         e     = edge_vec(h_ref);
                const Vec3d         back  = -u;
                const double beta = std::atan2(e.cross(back).norm(), e.dot(back));

                const double angle = total_angle(h_ref);
                const double theta = std::fmod(beta + (0.5 * angle), angle);
                return {
                        .next = OnVertex{.he = h_ref, .theta = theta},
                        .dist = dist,
                        .f    = f,
                        .b    = b,
                        .db   = db,
                };
            }

            // Edge crossing: express u in the frame of he, then flip into opposite(he).
            // A border twin only throws if the walk continues past it.
            const HalfEdgeIndex twin = opposite(he, m);
            const auto [x, y]        = frame(he);
            const Vec2d d            = Vec2d(u.dot(x), u.dot(y)).normalized();
            assert(d.y() < 0.0);  // outbound from face(he)
            return {
                    .next = OnEdge{.he = twin, .t = 1.0 - t, .d = -d},
                    .dist = dist,
                    .f    = f,
                    .b    = b,
                    .db   = db,
            };
        }
    };
}  // namespace

Point
exponential_map(const Mesh& mesh, const TangentVector& v, Geodesic* geod) {
    if (mdv::condition::is_zero_norm(v.cartesian_vector()))
        return v.application_point();

    constexpr std::size_t max_crossings = 1'000'000;

    const ContinuationStepper stepper{.m = internal::cgal_mesh(mesh)};
    const double              len     = v.cartesian_vector().norm();
    const double              eps_len = 1e-12 * len;  // stop-vs-cross tie

    const auto finish = [&](const Point& res) -> Point {
        if (geod != nullptr) geod->emplace_back(position(mesh, res));
        return res;
    };

    if (geod != nullptr) geod->emplace_back(position(mesh, v.application_point()));

    ContinuationState state     = stepper.initial_state(v);
    double            remaining = len;
    for (std::size_t i = 0; i < max_crossings; ++i) {
        const Step step = std::visit(stepper, state);

        // Stop strictly before reaching the next state
        if (remaining < step.dist - eps_len) {
            if (const auto* s = std::get_if<AlongEdge>(&state)) {
                const double t = s->t + (remaining / stepper.edge_vec(s->he).norm());
                return finish(stepper.edge_point(s->he, t));
            }
            Vec3d b = (step.b + (remaining * step.db)).cwiseMax(0.0);
            b /= b.sum();
            return finish(Point(mesh, internal::from_cgal(step.f), b));
        }

        // Stop exactly at the next state
        if (remaining <= step.dist + eps_len)
            return finish(stepper.location(step.next));

        remaining -= step.dist;
        state = step.next;
        if (geod != nullptr && step.dist > 0.0)
            geod->emplace_back(position(mesh, stepper.location(state)));
    }

    throw std::runtime_error(
            fmt::format("exponential_map: exceeded {} crossings", max_crossings)
    );
}
}  // namespace mdv::mesh
