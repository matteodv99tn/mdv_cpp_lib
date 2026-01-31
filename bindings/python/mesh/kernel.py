from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

try:
    from ._mesh_impl import MeshKernel as _MeshKernelImpl  # type: ignore[import-not-found]
    from ._mesh_impl import PointVector  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _MeshKernelImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def distance_matrix(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def __call__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def find_pointset_max_lengthscale(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

    class PointVector(list):  # type: ignore[no-redef]
        pass


from .point import Point
from .vertex import Vertex


class MeshKernel:
    """Python wrapper for mesh kernel operations.

    This class provides kernel-based operations on meshes, including geodesic
    distance matrix computations and squared exponential kernel evaluation.

    Parameters
    ----------
    mesh : Mesh
        Mesh to operate on.
    """

    def __init__(self, mesh):
        """Initialize a MeshKernel wrapper.

        Parameters
        ----------
        mesh : Mesh
            Mesh to operate on.
        """
        self._kernel_impl = _MeshKernelImpl(mesh._mesh_impl)

    def distance_matrix(
        self,
        points1: Sequence[Point | Vertex],
        points2: Optional[Sequence[Point | Vertex]] = None,
    ) -> NDArray[np.float64]:
        """Evaluate the geodesic distance matrix between points.

        Parameters
        ----------
        points1 : Sequence[Point | Vertex]
            First list of points.
        points2 : Sequence[Point | Vertex], optional
            Second list of points. If None, uses ``points1`` (symmetric case).

        Returns
        -------
        np.ndarray
            Distance matrix of shape ``(len(points1), len(points2))``.
        """
        pts1_impl = self._to_point_impl_list(points1)
        if points2 is None:
            pts2_impl = pts1_impl
        else:
            pts2_impl = self._to_point_impl_list(points2)

        return np.asarray(self._kernel_impl.distance_matrix(pts1_impl, pts2_impl))

    def __call__(
        self,
        points1: Sequence[Point | Vertex],
        points2: Sequence[Point | Vertex],
        lengthscale: float = 1.0,
    ) -> NDArray[np.float64]:
        """Evaluate the squared exponential (RBF) kernel.

        Parameters
        ----------
        points1 : Sequence[Point | Vertex]
            First list of points.
        points2 : Sequence[Point | Vertex]
            Second list of points.
        lengthscale : float, default=1.0
            Length scale parameter.

        Returns
        -------
        np.ndarray
            Kernel matrix of shape ``(len(points1), len(points2))``.
        """
        pts1_impl = self._to_point_impl_list(points1)
        pts2_impl = self._to_point_impl_list(points2)
        return np.asarray(self._kernel_impl(pts1_impl, pts2_impl, lengthscale))

    @staticmethod
    def evaluate_squared_exponential(
        distance_matrix: ArrayLike,
        lengthscale: float = 1.0,
    ) -> NDArray[np.float64]:
        """Apply the squared exponential (RBF) kernel to a distance matrix.

        Parameters
        ----------
        distance_matrix : array_like, shape (N, M)
            Pre-computed geodesic distance matrix.
        lengthscale : float, default=1.0
            Length scale parameter.

        Returns
        -------
        np.ndarray
            Kernel matrix with the same shape as ``distance_matrix``.
        """
        from ._mesh_impl import evaluate_squared_exponential  # type: ignore[import-not-found]

        return np.asarray(
            evaluate_squared_exponential(distance_matrix, float(lengthscale))
        )

    def find_pointset_max_lengthscale(
        self,
        points: Sequence[Point | Vertex],
        num_steps: int = 30,
    ) -> float:
        """Find a heuristic lengthscale upper bound for a point set.

        Parameters
        ----------
        points : Sequence[Point | Vertex]
            Points on the mesh.
        num_steps : int, default=30
            Number of steps for the internal search.

        Returns
        -------
        float
            Estimated maximum lengthscale.
        """
        return self._kernel_impl.find_pointset_max_lengthscale(
            self._to_point_impl_list(points), num_steps
        )

    def _to_point_impl_list(
        self,
        input: Sequence[Point | Vertex],
    ) -> PointVector:
        """Convert Point/Vertex objects to a SWIG PointVector.

        Parameters
        ----------
        input : Sequence[Point | Vertex]
            Objects to convert.

        Returns
        -------
        PointVector
            SWIG PointVector containing the converted points.

        Raises
        ------
        ValueError
            If an input object cannot be converted to a Point.
        """

        def ensure_point(p: Vertex | Point) -> Point:
            if isinstance(p, Vertex):
                return Point(p)
            elif isinstance(p, Point):
                return p
            raise ValueError(
                f"Don't know how to convert type {type(p)} to mdv.mesh.Point"
            )

        res = [ensure_point(p)._point_impl for p in input]  # type: ignore[attr-defined]
        return PointVector(res)

    @staticmethod
    def find_matrix_max_lengthscale(
        distance_matrix: ArrayLike,
        num_iters: int = 20,
        initial_lengthscale: float = 1.0,
    ) -> float:
        """Find a heuristic lengthscale upper bound for a distance matrix.

        Parameters
        ----------
        distance_matrix : array_like, shape (N, M)
            Pre-computed geodesic distance matrix.
        num_iters : int, default=20
            Number of iterations for the internal search.
        initial_lengthscale : float, default=1.0
            Starting lengthscale.

        Returns
        -------
        float
            Estimated maximum lengthscale.
        """
        from ._mesh_impl import (  # type: ignore[import-not-found]
            find_matrix_max_lengthscale as _find_impl,
        )

        return _find_impl(distance_matrix, int(num_iters), float(initial_lengthscale))
