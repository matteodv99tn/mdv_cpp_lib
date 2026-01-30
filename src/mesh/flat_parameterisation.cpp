#include "mdv/mesh/flat_parameterisation.hpp"

#if MDV_CGAL_VERSION == 5
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#elif MDV_CGAL_VERSION == 6
#include <CGAL/AABB_traits_2.h>
#include <CGAL/AABB_triangle_primitive_2.h>
#endif
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/Surface_mesh_parameterization/LSCM_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/Two_vertices_parameterizer_3.h>
#include <Eigen/Core>

#include "mdv/macros.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/mesh.hpp"

namespace smp = CGAL::Surface_mesh_parameterization;

namespace mdv::mesh {


struct FlatParameterisation::FlatParameterisationImpl {
    // For the 2D mesh, we require a different kernel because the EPICK fails in
    // computing closest points on AABB trees
    using K2 = CGAL::Simple_cartesian<double>;
    using K3 = internal::CgalImpl::Kernel;

    using Mesh2       = CGAL::Surface_mesh<K2>;
    using Mesh3       = internal::CgalImpl::Mesh;
    using VertexIndex = Mesh3::Vertex_index;
    using FaceIndex   = Mesh3::Face_index;
    using HalfEdge    = Mesh3::Halfedge_index;


    // Parameterisation types
    using UvMap               = Mesh3::Property_map<VertexIndex, K3::Point_2>;
    using BorderParameteriser = smp::Two_vertices_parameterizer_3<Mesh3>;
    using Parameteriser       = smp::LSCM_parameterizer_3<Mesh3>;

    // AabbTree
#if MDV_CGAL_VERSION == 5
    using Triangle2List = std::vector<K3::Triangle_3>;
    using AabbPrimitive = CGAL::AABB_triangle_primitive<K3, Triangle2List::iterator>;
    using AabbTraits    = CGAL::AABB_traits<K3, AabbPrimitive>;
    using Aabb          = CGAL::AABB_tree<AabbTraits>;
#elif MDV_CGAL_VERSION == 6
    using Triangle2List = std::vector<K2::Triangle_2>;
    using AabbPrimitive = CGAL::AABB_triangle_primitive_2<K2, Triangle2List::iterator>;
    using AabbTraits    = CGAL::AABB_traits_2<K2, AabbPrimitive>;
    using Aabb          = CGAL::AABB_tree<AabbTraits>;
#endif

    FlatParameterisationImpl(::mdv::mesh::Mesh* mesh) : m(mesh->cgal()._mesh) {
        setup_uv_parameterisation();
        setup_aabb_tree();
    }

    MDV_NODISCARD Eigen::Vector2d
                  mesh_to_plane(const Eigen::Vector3d& q, const FaceIndex& f) const {
        const auto [v0, v1, v2] = get_vertices(f);
        const auto bs           = construct_barycentric<Eigen::Vector3d>(
                q, get_vertex3(v0), get_vertex3(v1), get_vertex3(v2)
        );
        return bs(0) * get_vertex2(v0) + bs(1) * get_vertex2(v1)
               + bs(2) * get_vertex2(v2);
    }

    MDV_NODISCARD Eigen::Vector2d
                  mesh_to_plane(const VertexIndex& v) const {
        return get_vertex2(v);
    }

    Eigen::Vector3d
    plane_to_mesh(const Eigen::Vector2d& query) {
#if MDV_CGAL_VERSION == 5
        const K3::Point_3 q{query(0), query(1), 0.0};
#elif MDV_CGAL_VERSION == 6
        const K2::Point_2 q{query(0), query(1)};
#endif
        const auto [qclose, tri] = aabb.closest_point_and_primitive(q);

        assert(std::distance(tris.begin(), tri)
               < std::distance(tris.begin(), tris.end()));
        const FaceIndex f(std::distance(tris.begin(), tri));

        const auto [v0, v1, v2] = get_vertices(f);
        const auto bs           = construct_barycentric<Eigen::Vector2d>(
                {qclose.x(), qclose.y()},
                get_vertex2(v0),
                get_vertex2(v1),
                get_vertex2(v2)
        );
        return bs(0) * get_vertex3(v0) + bs(1) * get_vertex3(v1)
               + bs(2) * get_vertex3(v2);
    }

    MDV_NODISCARD bool
    is_one_to_one() const {
        return Parameteriser{}.is_one_to_one_mapping(m, bhe, uv_map);
    }

    MDV_NODISCARD bool
    is_inside_face(const Eigen::Vector2d& uv) const {
#if MDV_CGAL_VERSION == 5
        const K3::Point_3 q{uv(0), uv(1), 0.0};
#elif MDV_CGAL_VERSION == 6
        const K2::Point_2 q{uv(0), uv(1)};
#endif
        const auto qc = aabb.closest_point(q);
        return CGAL::squared_distance(q, qc) < 1e-18;
    }

    MDV_NODISCARD Eigen::Vector2d
                  min_uv() const {
        return {aabb.bbox().min(0), aabb.bbox().min(1)};
    }

    MDV_NODISCARD Eigen::Vector2d
                  max_uv() const {
        return {aabb.bbox().max(0), aabb.bbox().max(1)};
    }

private:
    MDV_NODISCARD K2::Point_2
                  get_uv(const VertexIndex& v) const {
        const auto pt = uv_map[v];
        return {pt.x(), pt.y()};
    }

    MDV_NODISCARD std::array<VertexIndex, 3>
                  get_vertices(const FaceIndex& f) const {
        auto       he = halfedge(f, m);
        const auto v0 = target(he, m);
        he            = next(he, m);
        const auto v1 = target(he, m);
        he            = next(he, m);
        const auto v2 = target(he, m);
        return {v0, v1, v2};
    }

    MDV_NODISCARD Eigen::Vector3d
                  get_vertex3(const VertexIndex& v) const {
        return internal::convert(m.point(v));
    }

    MDV_NODISCARD Eigen::Vector2d
                  get_vertex2(const VertexIndex& v) const {
        const auto pt = uv_map[v];
        return {pt.x(), pt.y()};
    }

#if MDV_CGAL_VERSION == 5
    MDV_NODISCARD K3::Point_3
                  get_point2(const VertexIndex& v) const {
        const auto pt = uv_map[v];
        return {pt.x(), pt.y(), 0.0};
    }
#elif MDV_CGAL_VERSION == 6
    MDV_NODISCARD K2::Point_2
                  get_point2(const VertexIndex& v) const {
        const auto pt = uv_map[v];
        return {pt.x(), pt.y()};
    }
#endif

    void
    setup_uv_parameterisation() {
        uv_map   = m.add_property_map<VertexIndex, K3::Point_2>("v:uv").first;
        bhe      = CGAL::Polygon_mesh_processing::longest_border(m).first;
        auto err = smp::parameterize(m, Parameteriser{}, bhe, uv_map);
        if (err != smp::OK) {
            std::cerr << "Error: " << smp::get_error_message(err) << "\n";
            throw std::runtime_error("Unable to construct parameterisation!");
        }
    }

    template <typename Vector>
    static Eigen::Vector3d
    construct_barycentric(
            const Vector& q, const Vector& a, const Vector& b, const Vector& c
    ) {
        const Vector v0 = b - a;
        const Vector v1 = c - a;
        const Vector v2 = q - a;

        const double d00 = v0.dot(v0);
        const double d01 = v0.dot(v1);
        const double d11 = v1.dot(v1);
        const double d20 = v2.dot(v0);
        const double d21 = v2.dot(v1);

        const double denom = d00 * d11 - d01 * d01;
        const double v     = (d11 * d20 - d01 * d21) / denom;
        const double w     = (d00 * d21 - d01 * d20) / denom;
        const double u     = 1.0 - v - w;
        return Eigen::Vector3d{u, v, w};
    }

    void
    setup_aabb_tree() {
        tris.reserve(m.number_of_faces());
        for (const auto f : m.faces()) {
            const auto [v0, v1, v2] = get_vertices(f);
            tris.emplace_back(get_point2(v0), get_point2(v1), get_point2(v2));
        }
        aabb = Aabb(tris.begin(), tris.end());
    }

    Mesh3&   m;
    UvMap    uv_map;
    Aabb     aabb;
    HalfEdge bhe;

    Triangle2List tris;
};

FlatParameterisation::FlatParameterisation(Mesh& mesh) :
        _mesh(&mesh), _impl(new FlatParameterisationImpl(_mesh)) {
    _mesh->logger().info("Creating flat parameterisation for mesh {}", _mesh->name());
};

FlatParameterisation::FlatParameterisation(FlatParameterisation&& other) noexcept :
        _mesh(other._mesh), _impl(other._impl) {
    other._impl = nullptr;
}

FlatParameterisation&
FlatParameterisation::operator=(FlatParameterisation&& other) noexcept {
    if (this == &other) return *this;
    this->_mesh = other._mesh;
    this->_impl = other._impl;
    other._impl = nullptr;
    return *this;
}

FlatParameterisation::~FlatParameterisation() {
    delete _impl;
};

Eigen::Vector2d
FlatParameterisation::project(const Point& pt) const {
    using VertexIndex = FlatParameterisationImpl::VertexIndex;
    using FaceIndex   = FlatParameterisationImpl::FaceIndex;

    const auto* v_desc = pt.get_as<Point::PointOnVertexDescriptor>();
    if (v_desc != nullptr)
        return _impl->mesh_to_plane(VertexIndex{v_desc->vertex().id()});
    return _impl->mesh_to_plane(pt.position(), FaceIndex{pt.face().id()});
}

Point
FlatParameterisation::retrieve(const Eigen::Vector2d& uv) const {
    return Point::from_cartesian(*_mesh, _impl->plane_to_mesh(uv));
}

bool
FlatParameterisation::is_one_to_one_mapping() const {
    return _impl->is_one_to_one();
}

bool
FlatParameterisation::is_inside_mesh(const Eigen::Vector2d& uv) const {
    return _impl->is_inside_face(uv);
}

Eigen::Vector2d
FlatParameterisation::min_uv() const {
    return _impl->min_uv();
}

Eigen::Vector2d
FlatParameterisation::max_uv() const {
    return _impl->max_uv();
}

}  // namespace mdv::mesh
