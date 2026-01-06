#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cstdint>
#include <Eigen/Geometry>
#include <stdexcept>

#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/all.hpp>

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

    double
    vertex_total_angle(
            const internal::CgalImpl::VertexDescriptor& v,
            const internal::CgalImpl::Mesh&             m
    ) {
        double res = 0.0;
        for (const auto he : CGAL::halfedges_around_source(v, m)) {
            const Kernel::Point_3& p0 = m.point(CGAL::source(he, m));
            const Kernel::Point_3& p1 = m.point(CGAL::target(he, m));
            const Kernel::Point_3& p2 = m.point(CGAL::target(next(he, m), m));
            res += CGAL::approximate_angle(p1, p0, p2);  // CGAL returns in degs!
        }
        return res * M_PI / 180.0;  // Convert to radians
    }

    enum IntersectionType : std::uint8_t {
        TRIANGLE_VERTEX_INTERSECTION,
        TRIANGLE_EDGE_INTERSECTION
    };

    std::pair<Kernel::Point_3, IntersectionType>
    compute_intersection(const Kernel::Ray_3 ray, const Kernel::Triangle_3 tri) {
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
        throw std::runtime_error("propagate along vertex not implemented");
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
            const internal::CgalImpl::Mesh& mesh,
            Geodesic*                       geod
    ) {
        if (geod != nullptr) geod->emplace_back(internal::convert(p0));

        const Kernel::Ray_3                     ray(p0, vec);
        CGAL::Triangle_from_face_descriptor_map tri_gen(&mesh);
        const Kernel::Triangle_3                tri = get(tri_gen, f_id);

        const auto [pstar, pstar_type] = compute_intersection(ray, tri);

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
        return exponential_map_impl(pstar, next_face_id, v_next, mesh, geod);
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
    const auto [pfinal, f] = exponential_map_impl(p0, f_id, vec, mesh, geod);
    const auto res         = Point::from_cartesian(
            v.application_point().mesh(), internal::convert(pfinal)
    );
    if (geod != nullptr) geod->emplace_back(res.position());
    return res;
}
}  // namespace mdv::mesh
