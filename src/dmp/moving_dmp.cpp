#include "mdv/dmp/moving_dmp.hpp"

#include <range/v3/all.hpp>

#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/rhythmic_dmp.hpp"
#include "mdv/eigen_defines.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/mesh_utilities.hpp"
#include "mdv/mesh/tangent_vector.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/utils/logging.hpp"


namespace rs = ::ranges;
namespace rv = ::ranges::views;

namespace mdv {
using mesh::Geodesic;
using mesh::Mesh;
using mesh::Point;
using mesh::TangentVector;

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

// Dmp typedefs
using M        = riemann::MeshManifold;
using Emb      = riemann::MeshEmbedder;
using MeshDmp  = mdv::RhytmicDmp<M, Emb>;
using MeshDemo = mdv::Demonstration<M>;

namespace {
    std::vector<Point>
    eigen_to_meshpt(const Mesh& mesh, const Geodesic& geod) {
        return geod | rv::transform([&mesh](const Vec3 pt) {
                   return Point::from_cartesian(mesh, pt);
               })
               | rs::to_vector;
    }

    Geodesic
    meshpt_to_eigen(const std::vector<Point>& geod) {
        return geod | rv::transform([](const Point& pt) { return pt.position(); })
               | rs::to_vector;
    }

    std::vector<double>
    create_equispaced(
            const int    n_samples,
            const double min,
            const double max,
            const bool   half_open_set
    ) {
        const double den  = half_open_set ? double(n_samples) : double(n_samples - 1);
        const double step = (max - min) / den;

        return rv::iota(0, n_samples) | rv::transform([step](const int i) -> double {
                   return step * double(i);
               })
               | rs::to_vector;
    }

    Vec3
    normal_projection(const Vec3& v, const Vec3& n) {
        return v - v.dot(n) * n;
    }


}  // namespace

std::vector<Point>
generate_trajectory(
        const MovingDmpParameters params, Mesh& mesh, const Geodesic& centroid_path
) {
    const auto& logger = *get_default_logger();

    logger.info("Generating and learning simple circular demonstration");
    const auto flat_mesh = Mesh::from_file(mesh::create_flat(5.0));

    const auto generate_pos = [&flat_mesh](const double theta) -> Point {
        Vec3 pos{-std::sin(theta), std::cos(theta), 0.0};
        return Point::from_cartesian(flat_mesh, pos);
    };

    const auto thetas = create_equispaced(100, 0.0, 2 * M_PI, true);
    const auto path   = thetas | rv::transform(generate_pos) | rs::to_vector;
    const auto demo   = MeshDemo::builder(path.size())
                              .assign_position(path)
                              .velocity_automatic_differentiation()
                              .acceleration_automatic_differentiation()
                              .set_sampling_period(std::chrono::milliseconds(10))
                              .create();
    logger.debug("Demonstration created");

    MeshDmp dmp;
    dmp.tau     = 1.0;
    auto origin = Point::from_cartesian(flat_mesh, Vec3::Zero());
    dmp.embedding().setup_from_point_and_direction(origin, Vec3::UnitY());
    logger.debug("Embedding setup");

    dmp.learn(demo, 1.0, origin);
    logger.info("Dmp on plane learned");


    const double len        = mesh::length(centroid_path);
    const double trav_time  = len / params.linear_speed;
    const long   geod_steps = std::ceil(trav_time / (1e-3 * double(params.dt_ms)));
    logger.info(
            "Path length/speed {:3f}/{:3f} -> Travel time: {:3f}s ({} samples at {}ms)",
            len,
            params.linear_speed,
            trav_time,
            geod_steps,
            params.dt_ms
    );

    logger.debug("Creating equispaced");
    const auto ss     = create_equispaced(params.num_centroid_steps, 0.0, 1.0, false);
    logger.debug("Resamling");
    const auto g_path = mesh::geodesic_resample(centroid_path, ss);
    logger.debug("To point");
    const auto g_pt_path = eigen_to_meshpt(mesh, g_path);

    // Notation:
    //  i time index for simulation
    //  k time index for stepping of the geodesic path

    const long embedding_update_steps = (geod_steps / params.num_centroid_steps) + 1;
    const auto get_k = [&embedding_update_steps](const long i) -> std::size_t {
        return i / embedding_update_steps;
    };

    const auto get_direction = [g_path](std::size_t k) -> Vec3 {
        assert(k < g_path.size());

        // Lonely exception -- Query on last point
        const std::size_t end = g_path.size() - 1;
        if (k == end) return (g_path[end] - g_path[end - 1]).normalized();

        return (g_path[k + 1] - g_path[k]).normalized();
    };


    Vec3d      dir              = get_direction(0).normalized();
    const auto update_embedding = [&](const std::size_t i) -> std::size_t {
        auto k = get_k(i);
        dir += 1.1 * params.dt_ms * 1e-3 * (get_direction(k) - dir);
        dir.normalize();
        if (i % embedding_update_steps != 0) return k;
        ;
        assert(k < g_pt_path.size());
        const auto& gk = g_pt_path[k];
        dmp.embedding().setup_from_point_and_direction(gk, dir);


        if (mesh::location_type(gk) == mesh::ON_EDGE)
            logger.warn("Center #{} is on edge", k);
        if (mesh::location_type(gk) == mesh::ON_VERTEX)
            logger.warn("Center #{} is on vertex", k);
        return k;
    };

    logger.debug(
            "Finding initial position for integration - Radius: {}",
            params.circle_radius
    );
    const auto& g0 = g_pt_path.front();
    const Vec3  n0 = g0.face().normal();
    const Vec3  v0 = normal_projection(get_direction(0), n0);
    const Vec3  d0 = n0.cross(v0).normalized() * params.circle_radius;

    auto y = mesh::exponential_map(TangentVector::from_ambient_vector(g0, d0));
    M::TangentVector v = Vec3::Zero();
    update_embedding(0);


    std::size_t        k_g = 0;
    std::vector<Point> res;
    res.reserve(geod_steps*2);
    dmp.tau = params.dmp_tau;
    std::chrono::milliseconds dt(params.dt_ms);

    // for (std::size_t i = 0; i < params.dmp_tau * 1e-2; ++i){
    v.setZero();
    for (std::size_t i = 0; i < std::size_t(params.dmp_tau / 1e-2); ++i){
        res.emplace_back(y);
        k_g = update_embedding(0);

        const auto [ynew, vnew] = dmp.integrate_once(
                y, v, g_pt_path[k_g], params.circle_radius, dt, i * dt
        );
        y = ynew;
        v = vnew;
    }

    for (std::size_t i = 0; i < geod_steps; ++i) {
        res.emplace_back(y);
        if (i % params.print_every == 0) logger.debug("i = {}", i);

        k_g = update_embedding(i);

        const auto [ynew, vnew] = dmp.integrate_once(
                y, v, g_pt_path[k_g], params.circle_radius, dt, i * dt
        );
        y = ynew;
        v = vnew;
    }

    for (std::size_t i = 0; i < std::size_t(params.dmp_tau / 1e-2); ++i){
        res.emplace_back(y);
        k_g = update_embedding(geod_steps- 1);

        const auto [ynew, vnew] = dmp.integrate_once(
                y, v, g_pt_path[k_g], params.circle_radius, dt, (geod_steps + i) * dt
        );
        y = ynew;
        v = vnew;
    }


    return res;
}

std::vector<mdv::mesh::Point>
upsample_to_1khz(
        const Mesh& mesh, const std::vector<Point>& in_path, MovingDmpParameters params
) {
    if (params.dt_ms == 1) return in_path;

    const auto& logger = *get_default_logger();

    const long samples_to_add = params.dt_ms - 1;
    assert(samples_to_add > 1);

    logger.info("Upsampling factor: {}", samples_to_add);

    const std::vector<double> interp_coords =
            rv::iota(0, samples_to_add)
            | rv::transform([&samples_to_add](const long i) -> double {
                  return i / double(samples_to_add);
              })
            | rs::to_vector;
    assert(interp_coords.size() == samples_to_add);

    const auto upsample_segment =
            [&interp_coords](const Vec3 p0, const Vec3 p1) -> Geodesic {
        const Vec3 delta = p1 - p0;
        return interp_coords | rv::transform([&p0, &delta](const double s) -> Vec3 {
                   return p0 + s * delta;
               })
               | rs::to_vector;
    };

    const Geodesic downsampled_path =
            in_path
            | rv::transform([](const auto& pt) -> Vec3 { return pt.position(); })
            | rs::to_vector;

    const auto upsampled_path =
            downsampled_path | rv::sliding(2)
            | rv::transform([&upsample_segment](const auto& rng) -> Geodesic {
                  return upsample_segment(rng[0], rng[1]);
              }) | rs::to<std::vector<Geodesic>>;
    const Geodesic vec = rv::join(upsampled_path) | rs::to_vector;

    return vec | rv::transform([&mesh](const Vec3& pos) -> Point {
               return Point::from_cartesian(mesh, pos);
           })
           | rs::to_vector;
}

std::vector<Quat>
encode_orientation(const std::vector<Point>& in_path, const bool flip_orientation) {
    const double z_mult = flip_orientation ? -1.0 : 1.0;
    const auto compute_quaternion = [z_mult](const Point& pt) -> Quat {
        const Vec3 dx = -Vec3::UnitX();
        const Vec3 vz = z_mult * pt.face().normal();
        const Vec3 vx = (dx - dx.dot(vz) * vz).normalized();
        const Vec3 vy = vz.cross(vx);
        Mat3d      rot;
        rot.col(0) = vx;
        rot.col(1) = vy;
        rot.col(2) = vz;
        assert(condition::is_zero(rot.determinant() - 1.0));
        return Quat{rot};
    };
    std::vector<Quat> res = in_path | rv::transform(compute_quaternion) | rs::to_vector;

    for (std::size_t i = 0; i < res.size() - 1; ++i)
        if (res[i].coeffs().dot(res[i + 1].coeffs()) < 0.0) res[i + 1].coeffs() *= -1.0;
    return res;
}

std::vector<Quat>
filter_orientation(const std::vector<Quat>& qin, const std::size_t window_size) {
    using Vec4 = Eigen::Vector4d;

    std::vector<Quat> res;
    res.reserve(qin.size());

    for (std::size_t i = 0; i < qin.size(); ++i) {
        const std::size_t ws = std::min({window_size, i, qin.size() - i - 1});
        assert(ws <= window_size);

        Vec4 sum = Vec4::Zero();
        for (std::size_t j = i - ws; j < i + ws + 1; ++j) sum += qin[j].coeffs();
        res.emplace_back(sum.normalized());
    }
    assert(res.size() == qin.size());
    return res;
}

}  // namespace mdv
