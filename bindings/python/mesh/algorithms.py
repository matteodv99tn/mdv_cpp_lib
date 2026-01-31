from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike, NDArray

from . import _mesh_impl as _impl  # type: ignore[import-not-found,attr-defined]

if TYPE_CHECKING:
    from .geodesic import Geodesic
    from .mesh import Mesh


def length(geodesic: "Geodesic") -> float:
    """Return the length of a geodesic.

    Parameters
    ----------
    geodesic : Geodesic
        Geodesic to measure.

    Returns
    -------
    float
        Geodesic length.
    """
    return _impl.length(geodesic._geodesic_impl)  # type: ignore[attr-defined]


def solve_path(
    mesh: "Mesh",
    x0: ArrayLike,
    x1: ArrayLike,
    t: ArrayLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Solve a geodesic transport path between point sets.

    Parameters
    ----------
    mesh : Mesh
        Mesh on which the computation is performed.
    x0 : array_like, shape (N, 3)
        Starting points on the mesh.
    x1 : array_like, shape (N, 3)
        End points on the mesh.
    t : array_like, shape (T,)
        Time samples in ``[0, 1]``.

    Returns
    -------
    xt : np.ndarray, shape (T, N, 3)
        Points along the path.
    ut : np.ndarray, shape (T, N, 3)
        Tangent vectors along the path.

    Notes
    -----
    The implementation is backed by the SWIG bindings and expects inputs
    convertible to ``numpy.ndarray``.
    """
    x0 = np.asarray(x0, dtype=np.float64)
    x1 = np.asarray(x1, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)

    if x0.shape != x1.shape:
        raise ValueError("x0 and x1 must have the same shape.")
    if x0.ndim != 2 or x0.shape[1] != 3:
        raise ValueError("x0 and x1 must have shape (N, 3).")
    if t.ndim != 1:
        raise ValueError("t must have shape (T,).")

    n_points = x0.shape[0]
    n_times = t.shape[0]

    xt = np.zeros((n_times, n_points, 3), dtype=np.float64)
    ut = np.zeros((n_times, n_points, 3), dtype=np.float64)

    path_data = _impl.solve_path(mesh._mesh_impl, x0, x1, t)  # type: ignore[attr-defined]

    for i in range(n_points):
        xt[:, i, :] = path_data[i][0]
        ut[:, i, :] = path_data[i][1]

    return xt, ut


def multithreaded_exponential_map(
    mesh: Mesh, xs: np.ndarray, vs: np.ndarray
) -> np.ndarray:
    """
    Inputs:
        mesh: mesh on which doing computation
        xs : (N, 3) Tensors on the mesh. The starting point.
        vs : (N, 3) Tensors on the mesh. The tangent vector at each xs.
    Outputs:
        ys : (N, 3) Tensors of the exponential map
    """
    return _impl.multithreaded_exponential_map(mesh._mesh_impl, xs, vs)
