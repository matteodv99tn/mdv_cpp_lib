#include "mdv/mesh/algorithm.hpp"

#include <atomic>
#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/all.hpp>

#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/cgal_geodesic.hpp"
#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/conditions.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/point.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/utils/conditions.hpp"
#include "mdv/utils/logging_extras.hpp"

#include "BS_thread_pool.hpp"

// \cond DOXYGEN_IGNORE
using mdv::mesh::Geodesic;
using mdv::mesh::Mesh;
using mdv::mesh::TangentVector;
// \endcond

namespace rs = ::ranges;
namespace rv = ::ranges::views;

double
mdv::mesh::length(const Geodesic& geod) {
    if (geod.size() < 2) return 0.0;

    auto segment_length = [](const auto& rng) -> double {
        const Eigen::Vector3d p1    = rng[0];
        const Eigen::Vector3d p2    = rng[1];
        const Eigen::Vector3d delta = p2 - p1;
        const double          res   = delta.norm();
        return res;
    };

    const double sum = rs::accumulate(
            geod | rv::sliding(2) | rv::transform(segment_length), double{0.0}
    );
    return sum;
}

mdv::mesh::CartesianPoint
mdv::mesh::point_from_geodesic(
        const Geodesic& geod, const double s_input, const double* len
) {
    double       internal_len = 0.0;
    const double s_target     = std::clamp(s_input, 0.0, 1.0);

    if (len == nullptr) {
        internal_len = length(geod);
        len          = &internal_len;
    }

    double s_travelled = 0.0;

    for (auto i = 0; i < geod.size() - 1; ++i) {
        const Eigen::Vector3d delta = geod[i + 1] - geod[i];
        const double          si    = delta.norm() / (*len);
        if (s_travelled + si < s_target) {
            s_travelled += si;
        } else {
            const double s_left = s_target - s_travelled;
            return geod[i] + delta * s_left / si;
        }
    }

    return geod.back();
}

Geodesic
mdv::mesh::geodesic_resample(const Geodesic& geod, std::vector<double> coordinates) {
    Geodesic     res;
    const double len = length(geod);
    res.reserve(coordinates.size());
    rs::transform(coordinates, std::back_inserter(res), [geod, len](const double s) {
        return point_from_geodesic(geod, s, &len);
    });
    return res;
};

Eigen::MatrixXd
mdv::mesh::geodesic_resample(const Geodesic& geod, const Eigen::VectorXd& coordinates) {
    if (geod.size() < 2)
        throw std::runtime_error("Cannot resample a geodesic with size less then 2!");


    const double    len = length(geod);
    const long      T   = coordinates.rows();
    Eigen::MatrixXd res(T, 3);

    for (long j = 0; j < T - 1; ++j) {
        if (coordinates[j + 1] < coordinates[j])
            throw std::runtime_error("coordinates must be monotonically increasing!");
    }

    double s_travelled = 0.0;

    auto   g_curr = geod.cbegin();
    auto   g_next = geod.cbegin() + 1;
    double si     = (*g_next - *g_curr).norm() / len;

    for (long k = 0; k < T; ++k) {
        const double s = coordinates(k);

        while (s_travelled + si < s) {
            s_travelled += si;
            ++g_curr;
            ++g_next;
            if (g_curr == geod.cend())
                throw std::runtime_error("Reached goedesic end!");
            si = (*g_next - *g_curr).norm() / len;
        }

        const double sk = (s - s_travelled) / si;
        assert(sk >= 0.0);
        assert(sk <= 1.0 + 1e-9);
        if (sk > 1.0 + 1e-6) throw std::runtime_error("SK error");
        res.row(k) = *g_curr + (*g_next - *g_curr) * sk;
    }
    return res;
};

TangentVector
mdv::mesh::parallel_transport(
        const TangentVector& tangent_vector, const Point& dest_point
) {
    using namespace mdv::condition;
    using Mat3 = Eigen::Matrix3d;

    auto build_trihedron = [](const Point& pt, const Vec3d& dir) -> Mat3 {
        assert(is_unit_norm(dir));

        Mat3        res;
        const Vec3d n = pt.face().normal();
        res.col(0)    = dir;
        res.col(2)    = n;
        res.col(1)    = res.col(2).cross(res.col(0));

        assert(are_orthogonal(res.col(0), res.col(1)));
        assert(are_orthogonal(res.col(0), res.col(2)));
        assert(are_orthogonal(res.col(1), res.col(2)));
        assert(is_zero(res.determinant() - 1.0));

        return res;
    };

    assert(Mesh::default_logger);
    mdv::Logger& logger = *Mesh::default_logger.get();

    logger.debug(
            "Computing parallel transport of vector {} applied in {} to target "
            "point "
            "{}",
            eigen_to_str(tangent_vector.application_point().position()),
            eigen_to_str(tangent_vector.cartesian_vector()),
            eigen_to_str(dest_point.position())
    );

    const bool on_same_face =
            tangent_vector.application_point().face() == dest_point.face();
    const bool on_same_position = are_equal(
            tangent_vector.application_point().position(), dest_point.position()
    );
    if (on_same_face || on_same_position)
        return {dest_point, tangent_vector.cartesian_vector()};

    const Point& start_point = tangent_vector.application_point();

    const Geodesic geod = start_point.face().mesh().build_geodesic(
            tangent_vector.application_point(), dest_point
    );


    const std::size_t n       = geod.size();
    const Vec3d       x_start = (geod[1] - geod[0]).normalized();
    const Vec3d       x_dest  = (geod[n - 1] - geod[n - 2]).normalized();

    const Vec3d pt_start  = start_point.position();
    const Vec3d pt_dest   = dest_point.position();
    const Vec3d n_start   = start_point.face().normal();
    const Vec3d n_dest    = dest_point.face().normal();
    const Vec3d vec_start = tangent_vector.cartesian_vector();

    assert((geod[1] - geod[0]).norm() > 1e-10);
    assert((geod[n - 1] - geod[n - 2]).norm() > 1e-10);

    const auto R1 = build_trihedron(start_point, x_start);  // NOLINT
    const auto R2 = build_trihedron(dest_point, x_dest);    // NOLINT

    assert(are_parallel(R1.col(2), n_start));
    assert(are_parallel(R2.col(2), n_dest));


    const Mat3  Rot      = R2 * R1.transpose();
    const Vec3d vec_dest = Rot * vec_start;
    assert(are_orthogonal(vec_start, n_start));
    if (!mdv::condition::are_orthogonal(vec_dest, dest_point.face().normal())) {
        const Vec3d  nf = start_point.face().normal();
        const Vec3d  nt = dest_point.face().normal();
        const Vec3d  vf = tangent_vector.cartesian_vector();
        const double s1 = nf.dot(vf);
        const double s2 = nt.dot(vec_dest);
        std::cout << "from normal: " << nf.transpose() << "\n";
        std::cout << "to normal: " << nt.transpose() << "\n";
        std::cout << "Input: " << vf.transpose() << " -> " << s1 << "\n";
        std::cout << "Rot matrix: \n" << Rot << "\n";
        std::cout << "Output: " << vec_dest.transpose() << " -> " << s2 << "\n";
    }
    assert(are_orthogonal(vec_dest, n_dest));
    // return TangentVector::from_tip_position(
    //         dest_point, dest_point.position() + vec_dest
    // );
    const TangentVector res(dest_point, vec_dest);
    assert(are_parallel(res.cartesian_vector(), vec_dest));
    return res;
}

TangentVector
mdv::mesh::logarithmic_map(const Point& p, const Point& y) {
    using mdv::condition::are_equal;

    assert(Mesh::default_logger);
    mdv::Logger& logger = *Mesh::default_logger.get();
    logger.debug(
            "Computing logarithmic map of point {} w.r.t. point {}",
            eigen_to_str(y.position()),
            eigen_to_str(p.position())
    );

    if (are_equal(p.position(), y.position())) return {p, Eigen::Vector3d::Zero()};
    if (p.face() == y.face()) return {p, y.position() - p.position()};

    const auto geod        = p.face().mesh().build_geodesic(p, y);
    auto       log_map_dir = (geod[1] - geod[0]).normalized();
    auto       log_map_len = length(geod);
    return {p, Vec3d(log_map_len * log_map_dir)};
}

double
mdv::mesh::distance(const Face& f, const CartesianPoint& pt) {
    const Eigen::Vector3d delta = pt - f.half_edge().origin_position();
    return std::abs(delta.dot(f.normal()));
}

double
mdv::mesh::distance(const Point& p1, const Point& p2) {
    return (p1.position() - p2.position()).norm();
}

bool
mdv::mesh::uv_in_unitary_triangle(const Eigen::Vector2d& uv) {
    return (uv.sum() <= 1.0) && (uv(0) >= 0.0) && (uv(1) >= 0.0);
}

double
mdv::mesh::distance(const HalfEdge& he, const Eigen::Vector3d& p) {
    using Vec3         = Eigen::Vector3d;
    const Vec3   o     = he.origin_position();
    const Vec3   delta = p - o;
    const double dot   = he.normalised_direction().dot(delta);
    return (delta - dot * he.normalised_direction()).norm();
}

mdv::mesh::LocationType
mdv::mesh::location_type(const Point& pt) {
    if (std::holds_alternative<Point::PointInFaceDescriptor>(pt.descriptor())) {
        return INSIDE_FACE;
    }
    if (std::holds_alternative<Point::PointOnEdgeDescriptor>(pt.descriptor())) {
        return ON_EDGE;
    }
    return ON_VERTEX;
}

mdv::mesh::LocationType
mdv::mesh::location_type(const TangentVector& tv) {
    return location_type(tv.application_point());
}


BS::thread_pool th_pool;

std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>
mdv::mesh::solve_path(
        const Mesh&            mesh,
        const Eigen::MatrixXd& x0,
        const Eigen::MatrixXd& x1,
        const Eigen::VectorXd& t
) {
    using MatPair       = std::pair<Eigen::MatrixXd, Eigen::MatrixXd>;
    using MatPairVector = std::vector<MatPair>;

    assert(x0.rows() == x1.rows());
    const long N = x0.rows();
    const long T = t.rows();

    std::atomic_long n_trivials = 0;

    std::vector<MatPair> res(N);

    auto solve_path_index = [&x0, &x1, &t, &mesh, &N, &T, &res, &n_trivials](long i) {
        const auto pt0  = Point::from_cartesian(mesh, x0.row(i));
        const auto pt1  = Point::from_cartesian(mesh, x1.row(i));
        MatPair&   pair = res[i];

        if ((pt0.position() - pt1.position()).norm() < 1e-9) {
            ++n_trivials;
            pair.first  = pt0.position().transpose().replicate(T, 1);
            pair.second = Eigen::MatrixXd::Zero(T, 3);
            return;
        }

        const auto geod =
                internal::CgalGeodesicConstructor::threadlocal_geodesic(pt0, pt1);
        const double len = length(geod);
        pair.first       = geodesic_resample(geod, t);
        pair.second      = Eigen::MatrixXd(T, 3);

        for (long j = 0; j < T - 1; ++j) {
            pair.second.row(j) =
                    len * (pair.first.row(j + 1) - pair.first.row(j)).normalized();
        }
        pair.second.row(T - 1) = pair.second.row(T - 2);
    };

    for (long i = 0; i < N; ++i) 
        th_pool.detach_task([&solve_path_index, i]() { solve_path_index(i); });
    th_pool.wait();

    if (n_trivials > 2) {
        std::cout << "Number of trivial paths: " << n_trivials << " / " << N << "\n"
                  << std::flush;
    }
    return res;
}

Eigen::MatrixXd
mdv::mesh::multithreaded_exponential_map(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
) {
    using Vec3 = Eigen::Vector3d;

    if (xs.rows() != vs.rows() || xs.cols() != vs.cols())
        throw std::runtime_error("multithreaded exponential map with different sizes!");

    Eigen::MatrixXd res(xs.rows(), xs.cols());

    std::atomic_long n_zeroed = 0;
    std::atomic_long n_changed = 0;
    std::atomic_long n_vertices = 0;
    std::atomic_long n_vertices_after = 0;
    constexpr long batch_size = 16;

    auto process_row = [&res, &mesh, &xs, &vs, &n_zeroed, &n_changed, &n_vertices, &n_vertices_after](const long i) {
        const Vec3 pos{xs.row(i)};
        const Vec3 vec{vs.row(i)};
        const auto pt = Point::from_cartesian(mesh, pos);
        const auto tv = TangentVector::from_ambient_vector(pt, vec);

        if (!mdv::condition::is_zero_norm(vec) && tv.cartesian_vector().isZero())
            ++n_zeroed;

        if ((tv.cartesian_vector() - vec).norm() > 1e-6)
            ++n_changed;

        if (pt.get_as<Point::PointOnVertexDescriptor>() != nullptr)
            ++n_vertices;

        const auto exp = exponential_map(tv);
        res.row(i)     = exp.position();
        if(location_type(exp) == ON_VERTEX) ++n_vertices_after;
    };

    auto process_rows_batched = [&process_row, &xs](const long start) {
        const long end = std::min(start + batch_size, xs.rows());
        for (long i = start; i < end; ++i)
            process_row(i);
    };

    for (long i = 0; i < xs.rows(); i += batch_size) 
        th_pool.detach_task([&process_rows_batched, i] { process_rows_batched(i); });
    th_pool.wait();

    if (n_zeroed > 0) std::cout << "Zeroed " << n_zeroed << " vectors\n";
    if (n_changed > 2)
        fmt::print("\rExp Map | changed / num vs (after) / total : {} / {} ({}) / {}\n", n_changed, n_vertices, n_vertices_after, xs.rows());

    return res;
}

long
mdv::mesh::num_points_on_mesh(const Mesh& mesh, const Eigen::MatrixXd& xs) {
    using Vec3 = Eigen::Vector3d;

    const long N = xs.rows();
    long on_mesh = 0;
    for (long i = 0; i < N; ++i) {
        const Vec3 pos{xs.row(i)};
        const auto pt = Point::from_cartesian(mesh, pos);
        if ((pt.position() - pos).norm() < 1e-6)
            ++on_mesh;
    }
    return on_mesh;
}

Eigen::MatrixXd
mdv::mesh::projx(const Mesh& mesh, const Eigen::MatrixXd& xs) {
    using Vec3 = Eigen::Vector3d;
    if (xs.cols() != 3) throw std::runtime_error("projx require xs to have 3 columns");
    const long N = xs.rows();
    long on_v = 0;
    Eigen::MatrixXd res(N, 3);
    for (long i = 0; i < N; ++i) {
        const Vec3 xo{xs.row(i)};
        const auto cv = mesh.closest_vertex(xo);

        if ( (cv.position() - xo).norm() < 1e-6) {
            ++on_v;
            res.row(i) = xo;
            continue;
        }
        
        const auto pt = Point::from_cartesian(mesh, xo);
        res.row(i) = pt.position();
        if (location_type(pt) == ON_VERTEX) ++on_v;
    }

    return res;
}

Eigen::MatrixXd
mdv::mesh::proju(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
) {
    if (xs.cols() != 3) throw std::runtime_error("projx require xs to have 3 columns");
    Eigen::MatrixXd res(xs.rows(), xs.cols());
    for (long i = 0; i < xs.rows(); ++i) {
        const auto pt = Point::from_cartesian(mesh, xs.row(i));
        res.row(i) =
                TangentVector::from_ambient_vector(pt, vs.row(i)).cartesian_vector();
    }
    return res;
}

std::vector<Eigen::Matrix3d>
mdv::mesh::proj_transformation(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
) {
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;
    using Quat = Eigen::Quaterniond;

    if (xs.cols() != 3) throw std::runtime_error("projx require xs to have 3 columns");

    constexpr long batch_size = 16;
    const long N = xs.rows();
    std::vector<Mat3> res(N);

    const auto process_row = [&](const long i) {
        const Vec3 pos{xs.row(i)};
        const Vec3 vec{vs.row(i)};
        const auto pt = Point::from_cartesian(mesh, pos);
        const auto tv = TangentVector::from_ambient_vector(pt, vec);

        Mat3& tf = res[i];
        const Vec3& v1 = vec;
        const Vec3  v2 = tv.cartesian_vector();
        const Quat  q  = Quat::FromTwoVectors(v1, v2);
        tf = Mat3{q} * v2.norm() / v1.norm();
#if 1
        const Vec3 v3 = tf * v1;
        if ((v3-v2).norm() > 1e-9) 
            throw std::runtime_error("Computed wrong transformation matrix");
#endif
    };

    const auto process_rows_batched = [&process_row, N](const long start) {
        const long end = std::min(start + batch_size, N);
        for (long i = start; i < end; ++i)
            process_row(i);
    };

    for (long i = 0; i < xs.rows(); i += batch_size) 
        th_pool.detach_task([&process_rows_batched, i] { process_rows_batched(i); });
    th_pool.wait();


    return res;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
mdv::mesh::closest_face_normal_and_vertex(const Mesh& mesh, const Eigen::MatrixXd& xs) {
    if (xs.cols() != 3) throw std::runtime_error("projx require xs to have 3 columns");

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> res = std::make_pair(
            Eigen::MatrixXd(xs.rows(), xs.cols()), Eigen::MatrixXd(xs.rows(),
            xs.cols())
    );
    Eigen::MatrixXd& ns = res.first;
    Eigen::MatrixXd& vs = res.second;

    long n_singular = 0;
    for (long i = 0; i < xs.rows(); ++i) {
        const auto pt = Point::from_cartesian(mesh, xs.row(i));
        ns.row(i)     = pt.face().normal();
        const auto v  = mesh.vertex(pt.face().vertices_ids()[0]);
        vs.row(i)     = v.position();
        if (location_type(pt) != LocationType::INSIDE_FACE) ++n_singular;
    }
    // if (n_singular > 0)
    //     std::cout << "Number of singular points: " << n_singular << "\n";
    return res;
}

void 
mdv::mesh::validate_projx(
        const Mesh& mesh, 
        const Eigen::MatrixXd& xs, 
        const Eigen::MatrixXd& xs_proj
        ) {
    using Vec3 = Eigen::Vector3d;

    const long N = xs.rows();
    long n_vertices = 0;
    long n_changed = 0;
    long n_good_proj = 0;
    
    for (long i = 0; i < N; ++i) {
        const Vec3 pos{xs.row(i)};
        const auto pt = Point::from_cartesian(mesh, pos);
        const Vec3 proj{xs_proj.row(i)};

        if ((pt.position() - proj).norm() > 1e-6) ++n_changed;
        if (pt.get_as<Point::PointOnVertexDescriptor>() != nullptr) ++n_vertices;

        const auto pp = Point::from_cartesian(mesh, proj);
        if ((pp.position() - proj).norm() < 1e-6) ++n_good_proj;
    }

    fmt::print("\rProjx | changed / valid / n. verts / tot : {} / {} / {} / {}\n", n_changed, n_good_proj, n_vertices, N);
}
