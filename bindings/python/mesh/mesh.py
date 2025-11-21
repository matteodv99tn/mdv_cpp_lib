from .vertex import Vertex
from .face import Face
from .point import Point
from .geodesic import Geodesic
from ._mesh_impl import Mesh as _MeshImpl, load_from_file, mesh_directory

import numpy as np
import pymeshlab
import pyvista


class Mesh:
    """
    Python wrapper for mesh data structure.
    
    This class provides access to mesh data including vertices, faces,
    and geometric operations like geodesic path computation.
    
    Attributes
    ----------
    name : str
        Name of the mesh
    num_vertices : int
        Number of vertices in the mesh
    num_faces : int
        Number of faces in the mesh
    """

    def __init__(
        self,
        vertices: np.typing.ArrayLike | None = None,
        faces: np.typing.ArrayLike | None = None,
        mesh_impl: _MeshImpl | None = None,
    ):
        """
        Initialize a Mesh wrapper.
        
        This constructor supports two initialization methods:
        1. From vertices and faces arrays (using vertices and faces parameters)
        2. From existing SWIG mesh object (using mesh_impl parameter)
        
        Parameters
        ----------
        vertices : np.typing.ArrayLike, optional
            Array of vertex coordinates with shape (nv, 3) where nv is number of vertices
        faces : np.typing.ArrayLike, optional
            Array of face indices with shape (nf, 3) where nf is number of faces
        mesh_impl : _MeshImpl, optional
            The underlying SWIG mesh object
            
        Raises
        ------
        RuntimeError
            If vertices and faces are not provided when creating from arrays
            If mesh construction fails
            
        Notes
        -----
        When providing vertices and faces arrays, the mesh will be created using
        pymeshlab for processing and then loaded from a temporary OFF file.
        """
        import tempfile

        if mesh_impl is not None:
            self._mesh_impl = mesh_impl
            return

        if (faces is None) or (vertices is None):
            raise RuntimeError(
                "To construct a mesh, a nv x 3 matrix of vertices and"
                "nf x 3 matrix of faces is required")

        pymesh = pymeshlab.Mesh(vertex_matrix=vertices, face_matrix=faces)
        meshset = pymeshlab.MeshSet()
        meshset.add_mesh(pymesh)

        with tempfile.NamedTemporaryFile(suffix=".off", delete=False) as tmp:
            meshset.save_current_mesh(
                tmp.name,
                save_vertex_color=False,
                save_vertex_normal=False,
                save_face_color=False,
                save_polygonal=False,
            )
            self._mesh_impl = load_from_file(tmp.name)

    @staticmethod
    def load_from_file(file_path: str) -> 'Mesh':
        """
        Load a mesh from a file.
        
        Parameters
        ----------
        file_path : str
            Path to the mesh file
            
        Returns
        -------
        Mesh
            Loaded mesh object
        """
        return Mesh(mesh_impl=load_from_file(file_path))

    @staticmethod
    def mesh_directory() -> str:
        """
        Get the directory where mesh files are stored.
        
        Returns
        -------
        str
            Path to the mesh directory
        """
        return mesh_directory()

    @staticmethod
    def cube_angle() -> "Mesh":
        """
        Creates a mesh which represent 3 faces of a cube that are sharing the same 
        vertex.
        
        The cube has unitary dimension, and the shared vertex is located at (1, 1, 1).
        
        Returns
        -------
        Mesh
            A new Mesh object representing the cube
        """
        vertices = np.array(
            [
                [1.0, 0.0, 0.0],  # vertex 0
                [1.0, 1.0, 0.0],  # vertex 1
                [0.0, 1.0, 0.0],  # vertex 2
                [0.0, 1.0, 1.0],  # vertex 3
                [1.0, 1.0, 1.0],  # vertex 4
                [1.0, 0.0, 1.0],  # vertex 5
                [0.0, 0.0, 1.0],  # vertex 6
            ],
            dtype=np.float64,
        )
        faces = np.array(
            [[0, 1, 4], [0, 4, 5], [1, 2, 4], [2, 3, 4], [4, 3, 6], [4, 6, 5]],
            dtype=np.int32,
        )
        return Mesh(vertices=vertices, faces=faces)

    @property
    def num_vertices(self) -> int:
        """
        Get the number of vertices in the mesh.
        
        Returns
        -------
        int
            Number of vertices
        """
        return self._mesh_impl.num_vertices()

    @property
    def num_faces(self) -> int:
        """
        Get the number of faces in the mesh.
        
        Returns
        -------
        int
            Number of faces
        """
        return self._mesh_impl.num_faces()

    def get_vertex_matrix(self) -> np.typing.ArrayLike:
        """
        Get the vertex matrix of the mesh.
        
        Returns
        -------
        np.typing.ArrayLike
            An Nx3 matrix where each row represents a vertex position [x, y, z]
        """
        return self._mesh_impl.get_vertex_matrix()

    def get_face_matrix(self) -> np.typing.ArrayLike:
        """
        Get the face matrix of the mesh.
        
        Returns
        -------
        np.typing.ArrayLike
            An Nx3 matrix where each row represents face indices [v0, v1, v2]
            
        Notes
        -----
        This method returns face indices as doubles for Python binding compatibility.
        The underlying implementation uses get_face_matrix_double() which converts
        the integer indices to double precision for proper Python binding handling.
        """
        res: np.ndarray = self._mesh_impl.get_face_matrix_double()
        res = res.astype(np.int32)
        return res

    def vertex(self, id: int) -> Vertex:
        """
        Get a vertex by its identifier.
        
        Parameters
        ----------
        id : int
            Vertex identifier
            
        Returns
        -------
        Vertex
            Vertex with the specified identifier
        """
        return Vertex(self._mesh_impl.vertex(id))

    def face(self, id: int) -> Face:
        """
        Get a face by its identifier.
        
        Parameters
        ----------
        id : int
            Face identifier
            
        Returns
        -------
        Face
            Face with the specified identifier
        """
        return Face(self._mesh_impl.face(id))

    @property
    def faces(self) -> list[Face]:
        """
        Get all faces in the mesh.
        
        Returns
        -------
        list[Face]
            List of all Face objects in the mesh
        """
        return [self.face(i) for i in range(self.num_faces)]

    @property
    def vertices(self) -> list[Vertex]:
        """
        Get all vertices in the mesh.
        
        Returns
        -------
        list[Vertex]
            List of all Vertex objects in the mesh
        """
        return [self.vertex(i) for i in range(self.num_vertices)]

    def build_geodesic(self, from_point: Point, to_point: Point) -> Geodesic:
        """
        Build a geodesic path between two points on the mesh.
        
        Parameters
        ----------
        from_point : Point
            Starting point of the geodesic
        to_point : Point
            Ending point of the geodesic
            
        Returns
        -------
        Geodesic
            Geodesic path between the two points
        """
        return Geodesic(
            self._mesh_impl.build_geodesic(from_point._point_impl,
                                           to_point._point_impl))

    def midpoint_subdivide(self, iterations: int = 1) -> 'Mesh':
        """
        Subdivide the mesh using midpoint subdivision.
        
        This method applies midpoint subdivision to the mesh, increasing the number
        of faces and vertices by splitting each face into smaller faces.
        
        Parameters
        ----------
        iterations : int, default=1
            Number of subdivision iterations to perform
            
        Returns
        -------
        Mesh
            A new mesh object with subdivided faces
            
        Notes
        -----
        Each subdivision iteration increases the number of faces approximately by a factor of 4.
        The subdivision is performed using pymeshlab's midpoint subdivision algorithm.
        """
        import tempfile

        meshset = self._create_meshset()
        meshset.meshing_surface_subdivision_midpoint(iterations=iterations)

        with tempfile.NamedTemporaryFile(suffix=".off", delete=False) as tmp:
            meshset.save_current_mesh(
                tmp.name,
                save_vertex_color=False,
                save_vertex_normal=False,
                save_face_color=False,
                save_polygonal=False,
            )
            newmesh = Mesh.load_from_file(tmp.name)
            return newmesh

    def to_pyvista(self) -> pyvista.PolyData:
        fs = self.get_face_matrix()
        vs = self.get_vertex_matrix()
        return pyvista.make_tri_mesh(vs, fs)

    def _create_meshset(self) -> pymeshlab.MeshSet:
        """
        Create a pymeshlab MeshSet from this mesh.
        
        This private method converts the current mesh data into a pymeshlab MeshSet
        which can be used for mesh processing operations.
        
        Returns
        -------
        pymeshlab.MeshSet
            A MeshSet containing this mesh
            
        Notes
        -----
        This method is used internally by subdivision and other mesh processing
        operations that require pymeshlab's mesh processing capabilities.
        """
        pymesh = pymeshlab.Mesh(
            vertex_matrix=self.get_vertex_matrix(),
            face_matrix=self.get_face_matrix(),
        )
        meshset = pymeshlab.MeshSet()
        meshset.add_mesh(pymesh)
        return meshset
