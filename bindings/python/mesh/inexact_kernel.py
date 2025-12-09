import numpy as np

from ._mesh_impl import InexactMeshKernel as _InexactMeshKernelImpl
from ._mesh_impl import Point as _PointImpl
from ._mesh_impl import PointVector
from .point import Point
from .vertex import Vertex
from .kernel import MeshKernel
from typing import Optional


class InexactMeshKernel(MeshKernel):
    """
    Python wrapper for inexact mesh kernel operations.
    
    This class provides kernel-based operations on meshes with caching support
    for improved performance when computing distance matrices multiple times.
    It inherits from MeshKernel and adds additional functionality for caching
    and inexact computations.
    
    Parameters
    ----------
    mesh : Mesh
        The mesh object to perform kernel operations on
    """

    def __init__(self, mesh):
        """
        Initialize an InexactMeshKernel wrapper.
        
        Parameters
        ----------
        mesh : Mesh
            The mesh object to perform kernel operations on
            
        Notes
        -----
        Creates an inexact kernel object that operates on the given mesh,
        enabling distance and kernel computations with caching support.
        """
        self._kernel_impl = _InexactMeshKernelImpl(mesh._mesh_impl)

    def distance_matrix(self, points: list[Point]) -> np.ndarray:
        """
        Compute the geodesic distance matrix for a list of points.
        
        Parameters
        ----------
        points : list[Point]
            List of points to compute distances between
            
        Returns
        -------
        np.ndarray
            Distance matrix of shape (len(points), len(points))
            
        Notes
        -----
        This method computes the geodesic distances between all pairs of points
        on the mesh surface. The computation is cached for performance when
        the same set of points is used multiple times.
        """
        pts_impl = self._to_point_impl_list(points)
        return self._kernel_impl.distance_matrix(pts_impl)

    def set_points1(self, points1: list[Point]) -> None:
        """
        Set the first set of points for caching.
        
        Parameters
        ----------
        points1 : list[Point]
            First list of points to cache
            
        Notes
        -----
        This method sets the first set of points for caching. When this is called,
        subsequent distance matrix computations can use cached results for improved
        performance.
        """
        pts1_impl = self._to_point_impl_list(points1)
        self._kernel_impl.set_points1(pts1_impl)

    def __call__(self,
                 points: list[Point],
                 lengthscale: float = 1.0) -> np.ndarray:
        """
        Evaluate the squared exponential (RBF) kernel for cached points.
        
        Parameters
        ----------
        points : list[Point]
            List of points to compute kernel for
        lengthscale : float, default=1.0
            Length scale parameter for the RBF kernel
            
        Returns
        -------
        np.ndarray
            Kernel matrix of shape (len(points), len(points))
            
        Notes
        -----
        This method applies the squared exponential (RBF) kernel to pre-computed
        distances using cached computations for improved performance.
        """
        pts_impl = self._to_point_impl_list(points)
        return self._kernel_impl(pts_impl, lengthscale)
