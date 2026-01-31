from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from numpy.typing import NDArray

try:
    from ._mesh_impl import InexactMeshKernel as _InexactMeshKernelImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _InexactMeshKernelImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def distance_matrix(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def set_points1(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def find_pointset_max_lengthscale(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def __call__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")


from .kernel import MeshKernel
from .point import Point
from .vertex import Vertex


class InexactMeshKernel(MeshKernel):
    """Python wrapper for inexact mesh kernel operations.

    This class provides kernel-based operations on meshes with caching support
    for improved performance when computing distance matrices multiple times.
    It inherits from MeshKernel and adds additional functionality for caching
    and inexact computations.

    Parameters
    ----------
    mesh : Mesh
        Mesh to operate on.
    """

    def __init__(self, mesh):
        """Initialize an InexactMeshKernel wrapper.

        Parameters
        ----------
        mesh : Mesh
            Mesh to operate on.
        """
        self._kernel_impl = _InexactMeshKernelImpl(mesh._mesh_impl)  # type: ignore[attr-defined]

    def distance_matrix(
        self,
        points1: Sequence[Point | Vertex],
        points2: Sequence[Point | Vertex] | None = None,
    ) -> NDArray[np.float64]:
        """Compute the geodesic distance matrix for a list of points.

        Parameters
        ----------
        points1 : Sequence[Point | Vertex]
            Points to compute distances between.
        points2 : Sequence[Point | Vertex], optional
            Second list of points. If None, uses ``points1``.

        Returns
        -------
        np.ndarray
            Distance matrix of shape ``(len(points), len(points2))``.
        """
        pts_impl = self._to_point_impl_list(points1)  # type: ignore[attr-defined]
        if points2 is None:
            return np.asarray(self._kernel_impl.distance_matrix(pts_impl))

        pts2_impl = self._to_point_impl_list(points2)  # type: ignore[attr-defined]
        return np.asarray(self._kernel_impl.distance_matrix(pts_impl, pts2_impl))

    def set_points1(self, points1: Sequence[Point | Vertex]) -> None:
        """Set the first set of points for caching.

        Parameters
        ----------
        points1 : Sequence[Point | Vertex]
            First list of points to cache.
        """
        pts1_impl = self._to_point_impl_list(points1)  # type: ignore[attr-defined]
        self._kernel_impl.set_points1(pts1_impl)

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
            self._to_point_impl_list(points),  # type: ignore[attr-defined]
            num_steps,
        )

    def __call__(
        self,
        points: Sequence[Point | Vertex],
        points2: Sequence[Point | Vertex] | None = None,
        lengthscale: float = 1.0,
    ) -> NDArray[np.float64]:
        """Evaluate the squared exponential (RBF) kernel for cached points.

        Parameters
        ----------
        points : Sequence[Point | Vertex]
            Points to compute the kernel for.
        points2 : Sequence[Point | Vertex], optional
            Second list of points. If None, uses ``points``.
        lengthscale : float, default=1.0
            Length scale parameter.

        Returns
        -------
        np.ndarray
            Kernel matrix of shape ``(len(points), len(points2))``.
        """
        pts_impl = self._to_point_impl_list(points)  # type: ignore[attr-defined]
        if points2 is None:
            return np.asarray(self._kernel_impl(pts_impl, lengthscale))

        pts2_impl = self._to_point_impl_list(points2)  # type: ignore[attr-defined]
        return np.asarray(self._kernel_impl(pts_impl, pts2_impl, lengthscale))
