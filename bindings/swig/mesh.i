%module mesh

%{
#include "mdv/config.hpp"
#include "mdv/mesh/mesh.hpp"
#include "mdv/mesh/algorithm.hpp"
#include "mdv/mesh/kernel.hpp"
#include "mdv/mesh/gaussian_process.hpp"
#include "mdv/mesh/flat_parameterisation.hpp"
#include "mdv/mesh/tangent_vector.hpp"
%}

%include "std_string.i"
%include "std_vector.i"
%include "eigen.i"

%eigen_typemaps(Eigen::Vector2d)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::MatrixXi)

%init %{
import_array();
%}

%typemap(in) mdv::mesh::VertexId {
    unsigned long value = PyLong_AsUnsignedLong($input);
    if (PyErr_Occurred()) SWIG_fail;
    if (value > std::numeric_limits<mdv::mesh::Index>::max())
        SWIG_exception_fail(SWIG_OverflowError, "Vertex index out of range");
    $1 = $1_ltype{static_cast<mdv::mesh::Index>(value)};
}
%typemap(out) mdv::mesh::VertexId {
    $result = PyLong_FromUnsignedLong($1.value);
}
%apply mdv::mesh::VertexId { mdv::mesh::FaceId };
%apply mdv::mesh::VertexId { mdv::mesh::HalfEdgeId };
%typemap(out) std::array<mdv::mesh::VertexId, 3> {
    $result = PyTuple_New(3);
    if (!$result) SWIG_fail;
    for (std::size_t i = 0; i < 3; ++i) {
        PyObject* value = PyLong_FromUnsignedLong($1[i].value);
        if (!value) {
            Py_DECREF($result);
            $result = nullptr;
            SWIG_fail;
        }
        PyTuple_SET_ITEM($result, i, value);
    }
}
%apply std::array<mdv::mesh::VertexId, 3> { std::array<mdv::mesh::FaceId, 3> };
%rename(vertex_position) mdv::mesh::position(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
%rename(vertex_normal) mdv::mesh::normal(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
%rename(vertex_describe) mdv::mesh::describe(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
%rename(face_normal) mdv::mesh::normal(const mdv::mesh::Mesh&, mdv::mesh::FaceId);
%rename(face_describe) mdv::mesh::describe(const mdv::mesh::Mesh&, mdv::mesh::FaceId);
%rename(halfedge_face) mdv::mesh::face(const mdv::mesh::Mesh&, mdv::mesh::HalfEdgeId);
%rename(point_position) mdv::mesh::position(const mdv::mesh::Mesh&, const mdv::mesh::Point&);
%rename(point_face) mdv::mesh::face(const mdv::mesh::Mesh&, const mdv::mesh::Point&);
%rename(point_describe) mdv::mesh::describe(const mdv::mesh::Mesh&, const mdv::mesh::Point&);

namespace mdv::mesh {

    Eigen::Vector3d position(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
    Eigen::Vector3d normal(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
    std::string describe(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
    double total_angle(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
    double gauss_curvature(const mdv::mesh::Mesh&, mdv::mesh::VertexId);
    mdv::mesh::VertexId closest_vertex(const mdv::mesh::Mesh&, const Eigen::Vector3d&);

    Eigen::Vector3d normal(const mdv::mesh::Mesh&, mdv::mesh::FaceId);
    std::string describe(const mdv::mesh::Mesh&, mdv::mesh::FaceId);
    mdv::mesh::FaceId random_face(const mdv::mesh::Mesh&);
    std::array<mdv::mesh::VertexId, 3> vertices(const mdv::mesh::Mesh&, mdv::mesh::FaceId);
    std::array<mdv::mesh::FaceId, 3> neighbours(const mdv::mesh::Mesh&, mdv::mesh::FaceId);
    mdv::mesh::HalfEdgeId halfedge(const mdv::mesh::Mesh&, mdv::mesh::VertexId, mdv::mesh::FaceId);
    mdv::mesh::HalfEdgeId prev(const mdv::mesh::Mesh&, mdv::mesh::HalfEdgeId);
    mdv::mesh::HalfEdgeId twin(const mdv::mesh::Mesh&, mdv::mesh::HalfEdgeId);
    mdv::mesh::FaceId face(const mdv::mesh::Mesh&, mdv::mesh::HalfEdgeId);
    Eigen::Vector3d direction(const mdv::mesh::Mesh&, mdv::mesh::HalfEdgeId);
    Eigen::Vector3d inbound_direction(const mdv::mesh::Mesh&, mdv::mesh::HalfEdgeId);

    class Mesh {
        Mesh(gsl::owner<CgalImpl*> data, const std::string& name);
    public:
        std::string name() const;

        void transform(const Eigen::Affine3d& transformation);
        
        void scale(double);
        
        std::size_t num_faces() const;

        std::size_t num_vertices() const;


        Eigen::MatrixXd get_vertex_matrix() const;

        Eigen::MatrixXd get_face_matrix_double() const;

    };

    class Point {
    public:
        bool is_undefined() const;
    };

    mdv::mesh::Point from_cartesian(const mdv::mesh::Mesh&, const Eigen::Vector3d&);
    mdv::mesh::Point random_point(const mdv::mesh::Mesh&);
    Eigen::Vector3d position(const mdv::mesh::Mesh&, const mdv::mesh::Point&);
    mdv::mesh::FaceId face(const mdv::mesh::Mesh&, const mdv::mesh::Point&);
    std::string describe(const mdv::mesh::Mesh&, const mdv::mesh::Point&);
    std::vector<Eigen::Vector3d> build_geodesic(const mdv::mesh::Mesh&, const mdv::mesh::Point&, const mdv::mesh::Point&);

    class TangentVector {
    public:
        enum Type : std::uint8_t {
            INSIDE_FACE,
            ALONG_EDGE
        };

        TangentVector(const mdv::mesh::Mesh&, const mdv::mesh::Point& app_point, const Eigen::Vector3d& v);

        static mdv::mesh::TangentVector from_ambient_vector(
                const mdv::mesh::Mesh&, const mdv::mesh::Point& application_point, const Eigen::Vector3d& vec
        );

        static mdv::mesh::TangentVector from_tip_position(
                const mdv::mesh::Mesh&, const mdv::mesh::Point& application_point, const Eigen::Vector3d& tip
        );

        static mdv::mesh::TangentVector unit_random(
                const mdv::mesh::Mesh&, const mdv::mesh::Point& application_point
        );


        Eigen::Vector3d cartesian_vector() const;

        void scale(const double& factor);

        void normalise();

        mdv::mesh::TangentVector normalised() const;

        mdv::mesh::Point application_point() const;

        mdv::mesh::TangentVector::Type type() const;
    };

    Eigen::Vector3d tip(const mdv::mesh::Mesh&, const mdv::mesh::TangentVector&);

    double length(const std::vector<Eigen::Vector3d>& geod);

    Eigen::Vector3d point_from_geodesic(
            const std::vector<Eigen::Vector3d>& geod, double s
    );

    std::vector<Eigen::Vector3d> geodesic_resample(
            const std::vector<Eigen::Vector3d>& geod, std::vector<double> coordinates
    );

    Eigen::MatrixXd geodesic_resample(
            const std::vector<Eigen::Vector3d>& geod, const Eigen::VectorXd& coordinates
    );


    mdv::mesh::TangentVector parallel_transport(
            const mdv::mesh::Mesh&, const mdv::mesh::TangentVector& v, const mdv::mesh::Point& p
    );

    mdv::mesh::TangentVector logarithmic_map(
            const mdv::mesh::Mesh&, const mdv::mesh::Point& p, const mdv::mesh::Point& y
    );

    mdv::mesh::Point exponential_map(
            const mdv::mesh::Mesh&, const mdv::mesh::TangentVector& v
    );

    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>> solve_path(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& x0,
            const Eigen::MatrixXd& x1,
            const Eigen::VectorXd& t
    );

    Eigen::MatrixXd multithreaded_exponential_map(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& xs,
            const Eigen::MatrixXd& vs
    );

    Eigen::MatrixXd projx(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& xs
    );

    void validate_projx(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd&,
            const Eigen::MatrixXd&
    );

    long num_points_on_mesh(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd&
    );

    Eigen::MatrixXd proju(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& xs,
            const Eigen::MatrixXd& vs
    );

    std::vector<Eigen::Matrix3d> proj_transformation(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& xs,
            const Eigen::MatrixXd& vs
    );

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> proj_transformation_directions(
            const mdv::mesh::Mesh& mesh,
            const Eigen::MatrixXd& xs,
            const Eigen::MatrixXd& vs
    );

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> closest_face_normal_and_vertex(
        const mdv::mesh::Mesh&, 
        const Eigen::MatrixXd&
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

        bool has_cache_support() const;

        double find_max_lengthscale(
                double ls0, double lsmax, std::size_t num_points, std::size_t num_tests, bool verbose
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

        bool has_cache_support() const;

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

    class FlatParameterisation {
    public:
        FlatParameterisation(mdv::mesh::Mesh&);
        bool is_one_to_one_mapping();
        Eigen::Vector2d project(const mdv::mesh::Point&);
        Point retrieve(const Eigen::Vector2d&);
        bool is_inside_mesh(const Eigen::Vector2d&);
        Eigen::Vector2d min_uv();
        Eigen::Vector2d max_uv();
    };

    class InexactGaussianProcess {
    public:
        InexactGaussianProcess(const mdv::mesh::Mesh* mesh, double lengthscale, double sigma_noise = 0.0);

        void train(const std::vector<mdv::mesh::Point>& in_points, const Eigen::MatrixXd& y_ref);

        Eigen::MatrixXd predict(const std::vector<mdv::mesh::Point>& test_points);
    };
}

%inline %{
    mdv::mesh::Point point_from_vertex(const mdv::mesh::Mesh& mesh, mdv::mesh::VertexId v) {
        (void)mdv::mesh::position(mesh, mdv::mesh::Point{mdv::mesh::PointOnVertex{v}});
        return mdv::mesh::Point{mdv::mesh::PointOnVertex{v}};
    }

    mdv::mesh::Mesh* load_from_file(const char* file) {
        return new mdv::mesh::Mesh(mdv::mesh::Mesh::from_file(std::filesystem::path(file)));
    }

    mdv::mesh::Mesh* extract_normal_bounded_surface(
                const mdv::mesh::Mesh& mesh, 
                const mdv::mesh::Point& pt, 
                const double a
   ){
       return new mdv::mesh::Mesh(mdv::mesh::Mesh::extract_normal_bounded_surface(mesh, pt, a));
   }

    mdv::mesh::Mesh* fill_holes(const mdv::mesh::Mesh& mesh){
        return new mdv::mesh::Mesh(mdv::mesh::Mesh::fill_holes(mesh));
    }

    std::string mesh_directory() {
        return mdv::config::meshes_directory();
    }

    Eigen::MatrixXd evaluate_squared_exponential(const Eigen::MatrixXd& d_mat, const double ls) {
        return mdv::mesh::internal::eval_sek(d_mat, ls);
    }

    bool is_positive_definite(const Eigen::MatrixXd& mat) {
        return mdv::mesh::internal::is_positive_definite(mat);
    }

    void apply_transform(mdv::mesh::Mesh& mesh, const Eigen::Matrix4d& t) {
        Eigen::Affine3d transform(t);
        mesh.transform(transform);
    }

    Eigen::MatrixXd geodesic_resample_matrix(
            const std::vector<Eigen::Vector3d>& geod, const Eigen::VectorXd& coordinates
    ) {
        return mdv::mesh::geodesic_resample(geod, coordinates);
    }

    mdv::mesh::InexactGaussianProcess* create_inexact_gaussian_process(
            const mdv::mesh::Mesh& mesh, double lengthscale, double sigma_noise
    ) {
        return new mdv::mesh::InexactGaussianProcess(&mesh, lengthscale, sigma_noise);
    }

    double find_matrix_max_lengthscale(const Eigen::MatrixXd& d, std::size_t n, double ls){
        return mdv::mesh::find_matrix_max_lengthscale(d, n, ls);
    }
%}

%newobject mdv::mesh::extract_normal_bounded_surface;
%newobject mdv::mesh::load_from_file;
%newobject mdv::mesh::fill_holes;
%newobject mdv::mesh::exponential_map;
%newobject mdv::mesh::geodesic_resample;
%newobject mdv::mesh::geodesic_resample_matrix;
%newobject mdv::mesh::create_inexact_gaussian_process;

%typemap(out) mdv::mesh::Mesh {
    $result = SWIG_NewPointerObj((new $1_ltype(std::move($1))), $&1_descriptor, SWIG_POINTER_OWN);
}

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
%template(Mat3Vector) std::vector<Eigen::Matrix3d>;
%template(PointVector) std::vector<mdv::mesh::Point>;
%template(VectorMatrixPairs) std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXd>>;
