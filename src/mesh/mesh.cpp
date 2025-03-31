#include "mdv/mesh/mesh.hpp"

#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <Eigen/Geometry>
#include <gsl/assert>
#include <spdlog/spdlog.h>
#include <string_view>

#include "mdv/mesh/cgal_impl.hpp"
#include "mdv/mesh/fwd.hpp"
#include "mdv/utils/logging.hpp"
#include "mdv/utils/logging_extras.hpp"


using mdv::mesh::Face;
using mdv::mesh::Mesh;
using mdv::mesh::Vertex;
using mdv::mesh::internal::CgalImpl;
using std::filesystem::path;

//   ____                _                   _
//  / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __ ___
// | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__/ __|
// | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |  \__ \
//  \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|  |___/
//
Mesh
Mesh::from_file(const std::filesystem::path& file_path) {
    const std::string     file_name = file_path.stem().string();
    auto                  logger    = class_logger_factory("Mesh", file_name, Debug);
    gsl::owner<CgalImpl*> data      = CgalImpl::from_file(file_path, std::move(logger));
    return Mesh(data, file_name);
}

Mesh::Mesh(gsl::owner<CgalImpl*> cgal_data, const std::string& name) {
    using Index        = Face::Index;
    using IndexTriplet = Face::IndexTriplet;

    Expects(cgal_data != nullptr);
    Expects(cgal_data->_logger != nullptr);
    _data.impl   = cgal_data;
    _data.logger = cgal_data->_logger;
    logger().info("Number of vertices: {}", cgal()._mesh.num_vertices());
    logger().info("Number of faces: {}", cgal()._mesh.num_faces());

    // Update Eigen-based data view
    _data.eigen_data = EigenData(*cgal_data, *_data.logger);
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

void
Mesh::transform(const Eigen::Affine3d& transformation) {
    logger().info("Applying transformation to mesh");
    const double m11 = transformation(0, 0);
    const double m12 = transformation(0, 1);
    const double m13 = transformation(0, 2);
    const double m14 = transformation(0, 3);
    const double m21 = transformation(1, 0);
    const double m22 = transformation(1, 1);
    const double m23 = transformation(1, 2);
    const double m24 = transformation(1, 3);
    const double m31 = transformation(2, 0);
    const double m32 = transformation(2, 1);
    const double m33 = transformation(2, 2);
    const double m34 = transformation(2, 3);

    const Mesh::CgalImpl::Transform transform(
            m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, 1.0
    );
    CGAL::Polygon_mesh_processing::transform(transform, cgal()._mesh);
}

Mesh::Geodesic
Mesh::build_geodesic(const Point& from, const Point& to) const {
    logger().debug(
            "Building geodesic from {} to {}",
            eigen_to_str(from.position()),
            eigen_to_str(to.position())
    );
    return internal::construct_geodesic(cgal(), from, to);
}

//   ____      _   _
//  / ___| ___| |_| |_ ___ _ __ ___
// | |  _ / _ \ __| __/ _ \ '__/ __|
// | |_| |  __/ |_| ||  __/ |  \__ \
//  \____|\___|\__|\__\___|_|  |___/
//

std::size_t
Mesh::num_vertices() const {
    return cgal()._mesh.num_vertices();
}

std::size_t
Mesh::num_faces() const {
    return cgal()._mesh.num_faces();
}

Mesh::FaceIterator
Mesh::faces_begin() const noexcept {
    return {&_data, 0};
}

Mesh::FaceIterator
Mesh::faces_end() const noexcept {
    return {&_data, static_cast<long>(num_faces())};
}

boost::iterator_range<Mesh::FaceIterator>
Mesh::faces() const noexcept {
    return {faces_begin(), faces_end()};
}

Mesh::VertexIterator
Mesh::vertices_begin() const noexcept {
    return {&_data, 0};
}

Mesh::VertexIterator
Mesh::vertices_end() const noexcept {
    return {&_data, static_cast<long>(num_vertices())};
}

boost::iterator_range<Mesh::VertexIterator>
Mesh::vertices() const noexcept {
    return {vertices_begin(), vertices_end()};
}
