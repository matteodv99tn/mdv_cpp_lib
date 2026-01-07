#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cstdint>
#include <Eigen/Geometry>
#include <limits>
#include <stdexcept>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/conditions.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/conditions.hpp"

// \cond DOXYGEN_IGNORE

// Uncomment the following line to enable the compilation with the integrated rerun
// logger that will display all the steps of the shortest path computation #define
// DEBUG_EXPONENTIAL_MAP

#if defined(DEBUG_EXPONENTIAL_MAP) && defined(MDV_WITH_RERUN_SDK)
#define RERUN_DEBUG_ENABLED true
#else
#define RERUN_DEBUG_ENABLED false
#endif

#if RERUN_DEBUG_ENABLED
#include <thread>

#include <rerun.hpp>
#include <rerun/recording_stream.hpp>

#include "mdv/rerun.hpp"
#endif

// \endcond

namespace mdv::mesh {

namespace {
    using Kernel           = internal::CgalImpl::Kernel;
    using FaceIndex        = internal::CgalImpl::CgalFaceIndex;
    using VertexIndex      = internal::CgalImpl::CgalVertexIndex;
    using HalfEdgeIndex    = internal::CgalImpl::CgalHalfEdgeIndex;
    using VertexDescriptor = Point::PointOnVertexDescriptor;
    using CgalHalfEdge     = internal::CgalImpl::CgalHalfEdgeIndex;
    using CgalMesh         = internal::CgalImpl::Mesh;

#if RERUN_DEBUG_ENABLED
    rerun::RecordingStream rec("exponential_map_debug", "exponential_map");
    std::int64_t           iter_count  = 0;
    const Mesh*            logged_mesh = nullptr;
    mdv::RerunConverter    to_rr;

    ::rerun::components::Position3D
    to_rr_pos(const Kernel::Point_3& in) {
        return {static_cast<float>(in.x()),
                static_cast<float>(in.y()),
                static_cast<float>(in.z())};
    }

    ::rerun::components::Vector3D
    to_rr_vec(const Kernel::Vector_3& in) {
        return {static_cast<float>(in.x()),
                static_cast<float>(in.y()),
                static_cast<float>(in.z())};
    }

    void
    draw_triangle(
            const std::string&        path,
            const Kernel::Triangle_3& tri,
            const std::string&        label
    ) {
        const Vec3d tv0 = internal::convert(tri.vertex(0));
        const Vec3d tv1 = internal::convert(tri.vertex(1));
        const Vec3d tv2 = internal::convert(tri.vertex(2));
        rec.log(path,
                to_rr(std::vector{tv0, tv1, tv2, tv0}).with_labels(std::vector{label}));
    }

    void
    draw_point(
            const std::string& path, const Kernel::Point_3& pt, const std::string& label
    ) {
        auto drawable = ::rerun::archetypes::Points3D(std::vector{to_rr_pos(pt)})
                                .with_labels(std::vector{label});
        rec.log(path, drawable);
    }

    void
    draw_arrow(
            const std::string&      path,
            const Kernel::Point_3&  origin,
            const Kernel::Vector_3& direction,
            const std::string&      label
    ) {
        using ::rerun::components::Vector3D;
        auto drawable = ::rerun::archetypes::Arrows3D()
                                .with_vectors(std::vector{to_rr_vec(direction)})
                                .with_origins(std::vector{to_rr_pos(origin)})
                                .with_labels(std::vector{label});
        rec.log(path, drawable);
    }

    void
    draw_arrow(
            const std::string&     path,
            const Kernel::Point_3& origin,
            const Vec3d&           direction,
            const std::string&     label
    ) {
        draw_arrow(path, origin, internal::eigen_to_vec3(direction), label);
    }
#endif

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

    HalfEdgeIndex
    get_closest_halfedge(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const internal::CgalImpl::Mesh& m
    ) {
        double        dist = std::numeric_limits<double>::infinity();
        HalfEdgeIndex closest_he{invalid_index};
        for (const auto he : CGAL::halfedges_around_face(halfedge(f, m), m)) {
            const Kernel::Ray_3 ray(m.point(source(he, m)), m.point(target(he, m)));
            const double        this_d = CGAL::squared_distance(ray, p);
            if (this_d < dist) {
                dist       = this_d;
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
        const auto he = get_closest_halfedge(p, f, m);
        const auto v0 = m.point(source(he, m));
        const auto v1 = m.point(target(he, m));
        const auto d0 = v0 - p;
        const auto d1 = v1 - p;

        assert((d0 * v) * (d1 * v) < 0.0);

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

    /*
     * Given a vector "v" applied at point "p" on the edge of face "f" whose direction
     * is outbound the face, gives the projected inbound vector on the contiguous face.
     */
    std::pair<Kernel::Vector_3, FaceIndex>
    propagate_along_edge(
            const Kernel::Point_3           p,
            const FaceIndex                 f,
            const Kernel::Vector_3          v,
            const internal::CgalImpl::Mesh& m
    ) {
        using CGAL::Polygon_mesh_processing::compute_face_normal;
        const auto      he     = get_halfedge(f, p, m);
        const FaceIndex next_f = m.face(CGAL::opposite(he, m));

        constexpr Index invalid = -1;
        if (next_f == FaceIndex{invalid})
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
        const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(-b1, b2);
        const Vec3d              v_next_eigen = q * internal::convert(v);
        const auto               v_next = internal::vector3_from_eigen(v_next_eigen);

        using mdv::condition::are_orthogonal, mdv::condition::is_unit_norm;
        assert(are_orthogonal(b1, ax) && are_orthogonal(b1, n1) && is_unit_norm(b1));
        assert(are_orthogonal(b2, ax) && are_orthogonal(b2, n2) && is_unit_norm(b2));
        assert(are_orthogonal(v_next_eigen, n2));

        return std::make_pair(v_next, next_f);
    }

    /*
     * Given a vector "v" applied at point "p" which is a vertex of face "f" whose
     * direction is outbound the face, gives the projected inbound vector on the
     * contiguous face.
     */
    std::pair<Kernel::Vector_3, FaceIndex>
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
            return CGAL::approximate_sqrt(CGAL::squared_length(v));
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
        while (traversed_angle < target_angle) {
            const auto curr_he = *he_circ;
            ++he_circ;
            const auto next_he = *he_circ;
            traversed_angle +=
                    CGAL::approximate_angle(he_to_vec3(curr_he), he_to_vec3(next_he));
        }

        const auto              next_f       = face(*he_circ, m);
        const double            excess_angle = traversed_angle - target_angle;
        const auto              n            = convert(compute_face_normal(next_f, m));
        const auto              d            = convert(normalize(he_to_vec3(*he_circ)));
        const Eigen::AngleAxisd rot(excess_angle * M_PI / 180.0, n);
        const Vec3d             vec_dir = rot * d;

        assert(are_orthogonal(n, d));
        assert(are_orthogonal(vec_dir, n));
        assert(vector_inside_triangle(
                convert(he_to_vec3(*he_circ)),
                vec_dir,
                convert(he_to_vec3(*(he_circ--)))
        ));
        return std::make_pair(vector3_from_eigen(vec_dir * length(v)), next_f);
    }

    std::pair<Kernel::Vector_3, FaceIndex>
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

        const Kernel::Ray_3                     ray(p0, vec);
        CGAL::Triangle_from_face_descriptor_map tri_gen(&mesh);
        const Kernel::Triangle_3                tri = get(tri_gen, f_id);

        const auto [pstar, pstar_type] =
                compute_intersection(p0, f_id, vec, vec_type, mesh);


#if RERUN_DEBUG_ENABLED
        rec.set_time_sequence("iteration", iter_count);
        ++iter_count;
        draw_triangle("initial_face", tri, "Starting face");
        draw_point("p0", p0, "p0");
        draw_point("pstar", pstar, "pstar");
        draw_arrow("tv", p0, vec, "initial vector");
#endif

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

        const auto [v_next, next_face_id] =
                propagate_vector(pstar_type, pstar, f_id, v_left, mesh);

#if RERUN_DEBUG_ENABLED
        const Kernel::Triangle_3 tri_next = get(tri_gen, next_face_id);

        draw_triangle("next_face", tri_next, "next face");
        draw_arrow("n1", pstar, n1, "initial face normal");
        draw_arrow("n2", pstar, n2, "target face normal");
        draw_arrow("b1", pstar, b1, "initial face binormal");
        draw_arrow("b2", pstar, b2, "target face binormal");
        draw_arrow("v1", pstar, internal::convert(v_left), "vector");
        draw_arrow("v2", pstar, v_next_eigen, "next vector");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif


        // TODO: check that the updated vector points "internally" to the face
        return exponential_map_impl(
                pstar, next_face_id, v_next, TangentVector::INSIDE_FACE, mesh, geod
        );
    }
}  // namespace

Point
exponential_map(TangentVector v, Geodesic* geod) {
#if RERUN_DEBUG_ENABLED
    rec.spawn().exit_on_failure();
    logged_mesh = &(v.application_point().mesh());
    rec.log_static("mesh", to_rr(v.application_point().mesh()));
    rec.set_time_sequence("iteration", iter_count);
    rec.log("tangent_vector", to_rr(std::vector{v}));
    ++iter_count;
#endif

    if (mdv::condition::is_zero_norm(v.cartesian_vector()))
        return v.application_point();

    const auto  p0         = internal::point3_from_point(v.application_point());
    const auto  f_id       = internal::to_face_impl(v.application_point().face());
    const auto& mesh       = internal::get_mesh_impl(v.application_point().face());
    const auto  vec        = internal::vector3_from_eigen(v.cartesian_vector());
    const auto [pfinal, f] = exponential_map_impl(p0, f_id, vec, v.type(), mesh, geod);
    const auto res         = Point::from_cartesian(
            v.application_point().mesh(), internal::convert(pfinal)
    );
    if (geod != nullptr) geod->emplace_back(res.position());
    return res;
}
}  // namespace mdv::mesh
