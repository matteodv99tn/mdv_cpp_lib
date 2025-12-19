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
%eigen_typemaps(Eigen::VectorXd)
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
        std::string name() const;
        
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

    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> solve_path(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& x0,
            const Eigen::MatrixXd& x1,
            const Eigen::VectorXd& t
    );

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

        double find_pointset_max_lengthscale(
                const std::vector<mdv::mesh::Point>& pts, std::size_t num_steps
        );
    };

    class InexactMeshKernel : public MeshKernel {
    public:
        InexactMeshKernel(const mdv::mesh::Mesh&);

        Eigen::MatrixXd distance_matrix(const std::vector<mdv::mesh::Point>& pts1, const std::vector<mdv::mesh::Point>& pts2);

        Eigen::MatrixXd distance_matrix(const std::vector<mdv::mesh::Point>& pts);

        void set_points1(const std::vector<mdv::mesh::Point>& pts1);

        Eigen::MatrixXd operator()(
                const std::vector<mdv::mesh::Point>& pts1, 
                const std::vector<mdv::mesh::Point>& pts2, 
                const double lengthscale
        );

        Eigen::MatrixXd operator()(
                const std::vector<mdv::mesh::Point>& pts, 
                const double lengthscale
        );

        double find_pointset_max_lengthscale(
                const std::vector<mdv::mesh::Point>& pts, std::size_t num_steps
        );
    };
}

%newobject load_from_file;

%inline %{
    mdv::mesh::Mesh* load_from_file(const char* file) {
        return new mdv::mesh::Mesh(mdv::mesh::Mesh::from_file(std::filesystem::path(file)));
    }

    std::string mesh_directory() {
        return mdv::config::meshes_directory();
    }

    Eigen::MatrixXd evaluate_squared_exponential(const Eigen::MatrixXd& d_mat, const double ls) {
        return mdv::mesh::internal::eval_sek(d_mat, ls);
    }

    double find_matrix_max_lengthscale(const Eigen::MatrixXd& d, std::size_t n, double ls){
        return mdv::mesh::find_matrix_max_lengthscale(d, n, ls);
    }
%}

%{
#include <utility>
%}

namespace std {
    template<class T, class U> struct pair {
        %extend {
            const T& __getitem__(int index) {
                if (index == 0) return $self->first;
                if (index == 1) return $self->second;
                throw std::out_of_range("pair index out of range");
            }
            
            int __len__() {
                return 2;
            }
        }
    };
}

%template(MatrixPair) std::pair<Eigen::MatrixXd, Eigen::MatrixXd>;
%template(Geodesic) std::vector<Eigen::Vector3d>;
%template(PointVector) std::vector<mdv::mesh::Point>;
%template(VectorMatrixPairs) std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>;
