#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sstream>
#include <string>

#ifdef MDV_WITH_RERUN_SDK
#include <rerun.hpp>
#include <rerun/archetypes/series_line.hpp>
#include <rerun/archetypes/series_point.hpp>
#endif  // MDV_WITH_RERUN_SDK

#include "mdv/config.hpp"
#include "mdv/containers/demonstration.hpp"
#include "mdv/dmp/dmp.hpp"
#include "mdv/dmp/dmp_utilities.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/rerun.hpp"
#include "mdv/riemann_geometry/mesh.hpp"
#include "mdv/utils/conversions.hpp"

using Quat = Eigen::Quaterniond;

std::string
to_string(const Quat& q) {
    std::stringstream ss;
    ss << q;
    return ss.str();
}

int
main() {
    using M     = mdv::riemann::MeshManifold;
    using Demo  = mdv::Demonstration<M>;
    using Dmp   = mdv::Dmp<M>;
    using Mesh  = mdv::mesh::Mesh;
    using Point = mdv::mesh::Mesh::Point;

    std::string mesh_path = std::string(mdv::config::mesh_dir) + "/torus_simple.off";
    fmt::print("Selected mesh path: {}\n", mesh_path);
    const auto mesh = Mesh::from_file(mesh_path);
    fmt::print("Mesh loaded\n");
    mesh.logger()->set_level(spdlog::level::info);


    using namespace std::chrono_literals;

    const long ns = 501;
    Point      y0 = Point::random(mesh);
    Point      g  = Point::random(mesh);


    Dmp dmp;
    dmp.tau        = 1.0;
    const Demo out = dmp.integrate(y0, g, ns, 2ms);

#ifdef MDV_WITH_RERUN_SDK
    //  ____  _       _   _   _
    // |  _ \| | ___ | |_| |_(_)_ __   __ _
    // | |_) | |/ _ \| __| __| | '_ \ / _` |
    // |  __/| | (_) | |_| |_| | | | | (_| |
    // |_|   |_|\___/ \__|\__|_|_| |_|\__, |
    //                                |___/
    using rerun::Scalar;
    using rerun::archetypes::SeriesLine;
    using rerun::archetypes::SeriesPoint;
    using rerun::components::Color;

    const Color c1(237, 135, 150);
    const Color c2(166, 218, 149);
    const Color c3(138, 173, 244);
    const Color c4(238, 212, 159);

    rerun::RecordingStream rec("meshdmp");
    rec.spawn().exit_on_failure();
    mdv::RerunConverter rr_converter;

    rec.log_static("mesh", rr_converter(mesh));

    mdv::mesh::Geodesic geod;
    geod.reserve(out.size());
    for (const auto& p : out) geod.emplace_back(p.y().position());

    rec.log_static("trajectory", rr_converter(geod).with_colors(c2));
    for (long i{0}; i < size(out); ++i) {
        // Export time
        rec.set_time_sequence("tick", i);
        rec.set_time_seconds("time", mdv::convert::seconds(out[i].t()));
        // Export posititon
        rec.log("position",
                rr_converter(out[i].y())
                        .with_colors(c1)
                        .with_radii(rerun::components::Radius::ui_points(5)));
    }
#endif  // MDV_WITH_RERUN_SDK

    return 0;
}
