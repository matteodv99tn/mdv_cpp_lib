%module mesh

%{
#include "mdv/config.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/kernel.hpp"
%}

%include "std_string.i"
%include "std_vector.i"
%include "eigen.i"

%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::MatrixXi)

%init %{
import_array();
%}

namespace mdv::mesh {

    class Vertex {
    public:
        Vertex(const mdv::mesh::Mesh&, const long int);

        Eigen::Vector3d position() const;

        Eigen::Vector3d normal();

        std::size_t id() const;

        std::string describe() const;

        double total_curvature() const;

        double gauss_curvature() const;
    };

    class Face {
    public:
        Face(const mdv::mesh::Mesh&, const long int);

        std::size_t id() const;

        Eigen::Vector3d normal();

        std::string describe() const;
    };

    class Mesh {
        Mesh(gsl::owner<CgalImpl*> data, const std::string& name);
    public:
        mdv::mesh::Vertex vertex(const long& id);

        std::size_t num_faces() const;

        std::size_t num_vertices() const;

        mdv::mesh::Face face(const long& id) const;

        std::vector<Eigen::Vector3d> build_geodesic(const mdv::mesh::Point& from, const mdv::mesh::Point& to) const;

        Eigen::MatrixXd get_vertex_matrix() const;

        // Eigen::MatrixXi get_face_matrix() const;
        Eigen::MatrixXd get_face_matrix_double() const;

        mdv::mesh::Vertex closest_vertex(const Eigen::Vector3d&);
    };

    class Point {
    public:
        Point(const mdv::mesh::Vertex& v);

        static mdv::mesh::Point from_cartesian(const mdv::mesh::Mesh& m, const Eigen::Vector3d& pt);

        Eigen::Vector3d position() const;

        static Point random(const mdv::mesh::Mesh& m) noexcept;

        mdv::mesh::Face face() const;

        std::string describe() const;
    };

    double length(const std::vector<Eigen::Vector3d>& geod);

    class MeshKernel {
    public:
        MeshKernel(const mdv::mesh::Mesh&);

        Eigen::MatrixXd distance_matrix(
                const std::vector<mdv::mesh::Point>& pts1, 
                const std::vector<mdv::mesh::Point>& pts2
        ) const;

        Eigen::MatrixXd operator()(
                const std::vector<mdv::mesh::Point>& pts1, 
                const std::vector<mdv::mesh::Point>& pts2, 
                const double lengthscale
        );
    };

    class InexactMeshKernel : public MeshKernel {
    public:
        InexactMeshKernel(const mdv::mesh::Mesh&);

        Eigen::MatrixXd distance_matrix(const std::vector<mdv::mesh::Point>& pts);

        void set_points1(const std::vector<mdv::mesh::Point>& pts1);

        Eigen::MatrixXd operator()(
                const std::vector<mdv::mesh::Point>& pts, 
                const double lengthscale
        );
    };
}

%inline %{
    mdv::mesh::Mesh load_from_file(const char* file) {
        return mdv::mesh::Mesh::from_file(std::filesystem::path(file));
    }

    std::string mesh_directory() {
        return mdv::config::meshes_directory();
    }

    Eigen::MatrixXd evaluate_squared_exponential(const Eigen::MatrixXd& d_mat, const double ls) {
        return mdv::mesh::internal::eval_sek(d_mat, ls);
    }
%}

%template(Geodesic) std::vector<Eigen::Vector3d>;
%template(PointVector) std::vector<mdv::mesh::Point>;
