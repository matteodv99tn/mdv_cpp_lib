from . import _mesh_impl as _impl

from typing import List
import numpy as np


class Vertex:
    """
    Python wrapper for mesh vertex.
    
    This class represents a vertex in a mesh, containing position, normal,
    and identification information.
    
    Attributes
    ----------
    position : np.ndarray
        3D position of the vertex
    normal : np.ndarray
        Normal vector at the vertex
    id : int
        Unique identifier of the vertex
    """

    def __init__(self, vertex_impl: _impl.Vertex):
        """
        Initialize a Vertex wrapper.

        > NOTE: this constructor is meant to be private.
        
        Parameters
        ----------
        vertex_impl : Vertex
            The underlying SWIG vertex object
        """
        self._impl = vertex_impl

    @property
    def position(self) -> np.ndarray:
        """
        Get the 3D position of the vertex.
        
        Returns
        -------
        np.ndarray
            3D position vector [x, y, z]
        """
        pos = self._impl.position()
        return np.array([pos.x(), pos.y(), pos.z()])

    @property
    def normal(self) -> np.ndarray:
        """
        Get the normal vector at the vertex.
        
        Returns
        -------
        np.ndarray
            Normal vector [x, y, z]
        """
        return self._impl.normal()

    @property
    def id(self) -> int:
        """
        Get the unique identifier of the vertex.
        
        Returns
        -------
        int
            Vertex identifier
        """
        return self._impl.id()

    def describe(self) -> str:
        """
        Get a string description of the vertex.
        
        Returns
        -------
        str
            Description of the vertex
        """
        return self._impl.describe()


class Face:
    """
    Python wrapper for mesh face.
    
    This class represents a face in a mesh, containing identification
    and normal information.
    
    Attributes
    ----------
    id : int
        Unique identifier of the face
    normal : np.ndarray
        Normal vector of the face
    """

    def __init__(self, face_impl: _impl.Face):
        """
        Initialize a Face wrapper.
        
        > NOTE: this constructor is meant to be private.

        Parameters
        ----------
        face_impl : Face
            The underlying SWIG face object
        """
        self._impl = face_impl

    @property
    def id(self) -> int:
        """
        Get the unique identifier of the face.
        
        Returns
        -------
        int
            Face identifier
        """
        return self._impl.id()

    @property
    def normal(self) -> np.ndarray:
        """
        Get the normal vector of the face.
        
        Returns
        -------
        np.ndarray
            Normal vector [x, y, z]
        """
        return self._impl.normal()

    def describe(self) -> str:
        """
        Get a string description of the face.
        
        Returns
        -------
        str
            Description of the face
        """
        return self._impl.describe()


class Point:
    """
    Python wrapper for mesh point.
    
    This class represents a point on a mesh, which can be defined either
    by its position in 3D space or by its location on a face using UV coordinates.
    
    Attributes
    ----------
    position : np.ndarray
        3D position of the point
    face : Face
        Face on which the point is located
    """

    def __init__(self, point_impl: _impl.Point):
        """
        Initialize a Point wrapper.

        > NOTE: this constructor is meant to be private.
        
        Parameters
        ----------
        point_impl : Point
            The underlying SWIG point object
        """
        self._impl = point_impl

    @property
    def position(self) -> np.ndarray:
        """
        Get the 3D position of the point.
        
        Returns
        -------
        np.ndarray
            3D position vector [x, y, z]
        """
        return self._impl.position()

    @property
    def face(self) -> Face:
        """
        Get the face on which the point is located.
        
        Returns
        -------
        Face
            The face containing this point
        """
        return Face(self._impl.face())

    def describe(self) -> str:
        """
        Get a string description of the point.
        
        Returns
        -------
        str
            Description of the point
        """
        return self._impl.describe()

    @staticmethod
    def from_cartesian(mesh: 'Mesh', position: np.ndarray) -> 'Point':
        """
        Create a point from cartesian coordinates.
        
        Parameters
        ----------
        mesh : Mesh
            The mesh to which the point belongs
        position : np.ndarray
            3D position vector [x, y, z]
            
        Returns
        -------
        Point
            New point created from cartesian coordinates
        """
        pos = Eigen_Vector3d(position[0], position[1], position[2])
        return Point(Point.from_cartesian(mesh._impl, pos))

    @staticmethod
    def random(mesh: 'Mesh') -> 'Point':
        """
        Create a random point on the mesh.
        
        Parameters
        ----------
        mesh : Mesh
            The mesh on which to generate the random point
            
        Returns
        -------
        Point
            Random point on the mesh
        """
        return Point(_impl.Point.random(mesh._impl))


class Geodesic:
    """ 
    Python wrapper of a geodesic
    """

    def __init__(self, geodesic_impl: _impl.Geodesic):
        self._impl = geodesic_impl

    def length(self) -> float:
        """
        Returns the length of the geodesic
        """
        return _impl.length(self._impl)

    def to_list(self) -> List[np.ndarray]:
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
        List[np.ndarray]
            List of 3D points representing the geodesic path
        """
        return [p for p in self._impl]


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

    def __init__(self, mesh_impl: _impl.Mesh):
        """
        Initialize a Mesh wrapper.

        > NOTE: this constructor is meant to be private.
        
        Parameters
        ----------
        mesh_impl : Mesh
            The underlying SWIG mesh object
        """
        self._impl = mesh_impl

    @property
    def num_vertices(self) -> int:
        """
        Get the number of vertices in the mesh.
        
        Returns
        -------
        int
            Number of vertices
        """
        return self._impl.num_vertices()

    @property
    def num_faces(self) -> int:
        """
        Get the number of faces in the mesh.
        
        Returns
        -------
        int
            Number of faces
        """
        return self._impl.num_faces()

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
        return Vertex(self._impl.vertex(id))

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
        return Face(self._impl.face(id))

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
        List[np.ndarray]
            List of 3D points representing the geodesic path
        """
        return Geodesic(
            self._impl.build_geodesic(from_point._impl, to_point._impl))

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
        return Mesh(_impl.load_from_file(file_path))


def mesh_directory() -> str:
    """
    Get the directory where mesh files are stored.
    
    Returns
    -------
    str
        Path to the mesh directory
    """
    return _impl.mesh_directory()


def length(geodesic: Geodesic) -> float:
    """
    Calculate the length of a geodesic path.
    
    Parameters
    ----------
    geodesic : Geodesic
        
    Returns
    -------
    float
        Length of the geodesic path
    """
    # Convert to SWIG format
    return _impl.length(geodesic._impl)
