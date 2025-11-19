from .vertex import Vertex
from .face import Face
from .point import Point
from .geodesic import Geodesic
from ._mesh_impl import Mesh as _MeshImpl, load_from_file, mesh_directory
import numpy as np


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
        import pymeshlab
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
        """
        return self._mesh_impl.get_face_matrix()

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
