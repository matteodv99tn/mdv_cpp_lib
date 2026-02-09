#ifndef MDV_MESH_ALGORITHM_HPP
#define MDV_MESH_ALGORITHM_HPP

#include "mdv/mesh/fwd.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/tangent_vector.hpp"

namespace mdv::mesh {

/**
 * @brief Computes the length of a geodesic polyline.
 *
 * @param geod Geodesic polyline.
 * @return Total length.
 */
double length(const Geodesic& geod);


/**
 * @brief Retrieves a point at a normalized curvilinear coordinate on a geodesic.
 *
 * s is clamped to [0, 1]. If len is provided, it is used to avoid recomputing the
 * geodesic length.
 *
 * @param geod Geodesic polyline.
 * @param s Normalized curvilinear coordinate.
 * @param len Optional pointer to precomputed geodesic length.
 * @return Point on the geodesic in ambient space.
 */
CartesianPoint point_from_geodesic(
        const Geodesic& geod, double s, const double* len = nullptr
);

/**
 * @brief Resamples a geodesic polyline at normalized coordinates.
 *
 * Coordinates are clamped to [0, 1].
 *
 * @param geod Geodesic polyline.
 * @param coordinates Normalized coordinates to sample.
 * @return Resampled geodesic polyline.
 */
Geodesic geodesic_resample(const Geodesic& geod, std::vector<double> coordinates);

/**
 * @brief Resamples a geodesic polyline into a T x 3 matrix.
 *
 * @param geod Geodesic polyline.
 * @param coordinates T-dimensional vector of normalized coordinates to sample.
 * @return Matrix of sampled points (T x 3).
 */
Eigen::MatrixXd geodesic_resample(
        const Geodesic& geod, const Eigen::VectorXd& coordinates
);

/**
 * @brief Parallel transports a tangent vector to a destination point.
 *
 * @param v Tangent vector to transport.
 * @param p Destination point on the mesh.
 * @return Transported tangent vector.
 */
TangentVector parallel_transport(const TangentVector& v, const Point& p);

/**
 * @brief Logarithmic map of y at p.
 *
 * @param p Base point on the mesh.
 * @param y Target point on the mesh.
 * @return Tangent vector at p pointing toward y.
 */
TangentVector logarithmic_map(const Point& p, const Point& y);

/**
 * @brief Exponential map of a tangent vector on the mesh.
 *
 * Optionally returns the unfolded geodesic path traced by the vector.
 *
 * @param v Tangent vector to apply.
 * @param geod Optional output geodesic polyline.
 * @return Point reached by the exponential map.
 */
Point exponential_map(TangentVector v, Geodesic* geod = nullptr);


/**
 * @brief Point-to-face distance (absolute distance to plane).
 *
 * @param f Face on the mesh.
 * @param pt Query point in 3D.
 * @return Absolute distance to the face plane.
 */
double distance(const Face& f, const CartesianPoint& pt);

/**
 * @brief Point-to-point distance in ambient 3D.
 *
 * @param p1 First point on the mesh.
 * @param p2 Second point on the mesh.
 * @return Euclidean distance in 3D.
 */
double distance(const Point& p1, const Point& p2);

/**
 * @brief Distance from a half-edge to a 3D point.
 *
 * @param he Half-edge on the mesh.
 * @param p Query point in 3D.
 * @return Euclidean distance to the edge line segment.
 */
double distance(const HalfEdge& he, const Eigen::Vector3d& p);

/**
 * @brief Checks if UV lies in the unit right triangle.
 *
 * @param uv UV coordinate.
 * @return True if inside or on the triangle.
 */
bool uv_in_unitary_triangle(const Eigen::Vector2d& uv);

/**
 * @brief Location type for a point on the mesh.
 */
enum LocationType : std::uint8_t {
    INSIDE_FACE = 0,
    ON_EDGE,
    ON_VERTEX
};

/**
 * @brief Returns the location type for a point.
 *
 * @param pt Point on the mesh.
 * @return Location type.
 */
LocationType location_type(const Point& pt);

/**
 * @brief Returns the location type for a tangent vector application point.
 *
 * @param tv Tangent vector.
 * @return Location type.
 */
LocationType location_type(const TangentVector& tv);

/**
 * @brief Solves geodesic paths and velocities for flow-matching on meshes.
 *
 * @param[in] mesh Surface mesh used for geodesics.
 * @param[in] x0 Nx3 matrix of starting points.
 * @param[in] x1 Nx3 matrix of ending points.
 * @param[in] t T-vector of normalized time indices in [0, 1].
 * @return Vector of N pairs: (Tx3 positions, Tx3 tangent velocities).
 */
std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> solve_path(
        const Mesh&            mesh,
        const Eigen::MatrixXd& x0,
        const Eigen::MatrixXd& x1,
        const Eigen::VectorXd& t
);

Eigen::MatrixXd multithreaded_exponential_map(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
);

Eigen::MatrixXd projx(const Mesh& mesh, const Eigen::MatrixXd& xs);

long num_points_on_mesh(const Mesh& mesh, const Eigen::MatrixXd& xs);

void 
validate_projx(const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& xs_proj);

Eigen::MatrixXd proju(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
);

std::vector<Eigen::Matrix3d> proj_transformation(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
);

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> proj_transformation_directions(
        const Mesh& mesh, const Eigen::MatrixXd& xs, const Eigen::MatrixXd& vs
);

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> closest_face_normal_and_vertex(
        const Mesh& mesh, const Eigen::MatrixXd& xs
);

}  // namespace mdv::mesh


#endif  // MDV_MESH_ALGORITHM_HPP
