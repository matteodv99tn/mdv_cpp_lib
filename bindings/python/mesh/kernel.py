import numpy as np

from ._mesh_impl import MeshKernel as _MeshKernelImpl
from ._mesh_impl import Point as _PointImpl
from ._mesh_impl import PointVector
from .point import Point
from .vertex import Vertex
from typing import Optional


class MeshKernel:
    """
    Python wrapper for mesh kernel operations.
    
    This class provides kernel-based operations on meshes, including distance matrix
    computations and kernel evaluations for machine learning applications.
    This class provides a faster API to compute distances and covariance matrices on 
    meshes between multiple points.
    
    Parameters
    ----------
    mesh : Mesh
        The mesh object to perform kernel operations on
    """

    def __init__(self, mesh):
        """
        Initialize a MeshKernel wrapper.
        
        Parameters
        ----------
        mesh : Mesh
            The mesh object to perform kernel operations on
            
        Notes
        -----
        Creates a kernel object that operates on the given mesh, enabling distance
        and kernel computations between points on the mesh surface.
        """
        self._kernel_impl = _MeshKernelImpl(mesh._mesh_impl)

    def distance_matrix(self,
                        points1: list[Point],
                        points2: Optional[list[Point]] = None) -> np.ndarray:
        """
        Evaluate the geodesic distance matrix between points on the mesh.
        
        Parameters
        ----------
        points1 : list[Point]
            First list of points
        points2 : list[Point], optional
            Second list of points. If None, uses points1 (symmetric case)
            
        Returns
        -------
        np.ndarray
            Distance matrix of shape (len(points1), len(points2))
            
        Notes
        -----
        The distance is computed as the geodesic distance along the mesh surface.
        If points2 is None, the result is a symmetric matrix.
        This method is useful for computing pairwise distances between points
        on the mesh for machine learning applications.
        """
        pts1_impl = self._to_point_impl_list(points1)
        if points2 is None:
            return self._kernel_impl.distance_matrix(pts1_impl)

        pts2_impl = self._to_point_impl_list(points2)
        return self._kernel_impl.distance_matrix(pts1_impl, pts2_impl)

    def __call__(self,
                 points1: list[Point],
                 points2: list[Point],
                 lengthscale: float = 1.0) -> np.ndarray:
        """
        Evaluate the squared exponential (RBF) kernel between points.
        
        Parameters
        ----------
        points1 : list[Point]
            First list of points
        points2 : list[Point]
            Second list of points
        lengthscale : float, default=1.0
            Length scale parameter for the RBF kernel
            
        Returns
        -------
        np.ndarray
            Kernel matrix of shape (len(points1), len(points2))
            
        Notes
        -----
        The squared exponential kernel is defined as:
        K(x1, x2) = exp(-0.5 * d(x1, x2)^2 / l^2)
        where d(x1, x2) is the geodesic distance and l is the length scale.
        
        This kernel is commonly used in Gaussian processes and kernel methods
        for mesh-based machine learning tasks.
        """
        pts1_impl = self._to_point_impl_list(points1)
        pts2_impl = self._to_point_impl_list(points2)
        return self._kernel_impl(pts1_impl, pts2_impl, lengthscale)

    @staticmethod
    def evaluate_squared_exponential(distance_matrix: np.ndarray,
                                     lengthscale: float = 1.0) -> np.ndarray:
        """
        Evaluate the squared exponential (RBF) kernel on a pre-computed distance matrix.
        
        Parameters
        ----------
        distance_matrix : np.ndarray
            Pre-computed geodesic distance matrix
        lengthscale : float, default=1.0
            Length scale parameter for the RBF kernel
            
        Returns
        -------
        np.ndarray
            Kernel matrix of shape (len(distance_matrix), len(distance_matrix))
            
        Notes
        -----
        This method applies the squared exponential (RBF) kernel directly to a 
        pre-computed distance matrix. This is useful when you already have 
        a distance matrix and want to apply the RBF kernel without recomputing
        the geodesic distances.
        
        The squared exponential kernel is defined as:
        K(x1, x2) = exp(-0.5 * d(x1, x2)^2 / l^2)
        where d(x1, x2) is the geodesic distance and l is the length scale.
        """

        from ._mesh_impl import evaluate_squared_exponential
        return evaluate_squared_exponential(distance_matrix,
                                            float(lengthscale))

    def _to_point_impl_list(self, input: list[Point | Vertex]) -> PointVector:
        """
        Convert a list of Point or Vertex objects to a PointVector for SWIG calls.
        
        Parameters
        ----------
        input : list[Point | Vertex]
            List of Point or Vertex objects to convert
            
        Returns
        -------
        PointVector
            SWIG PointVector containing the converted objects
            
        Raises
        ------
        ValueError
            If an input object cannot be converted to a Point
        """

        def ensure_point(p: Vertex | Point) -> Point:
            if isinstance(p, Vertex):
                return Point(p)
            elif isinstance(p, Point):
                return p
            raise ValueError(
                f"Don't know how to convert type {type(p)} to mdv.mesh.Point")

        res = [ensure_point(p)._point_impl for p in input]
        return PointVector(res)
