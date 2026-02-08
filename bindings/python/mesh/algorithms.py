from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike, NDArray

from . import _mesh_impl as _impl  # type: ignore[import-not-found,attr-defined]
from ._validation import as_vector

if TYPE_CHECKING:
    from .geodesic import Geodesic
    from .mesh import Mesh
    from .point import Point
    from .tangent_vector import TangentVector


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


def point_from_geodesic(geodesic: "Geodesic", s: float) -> NDArray[np.float64]:
    """Return a point on a geodesic at a normalized curvilinear coordinate.

    Parameters
    ----------
    geodesic : Geodesic
        Geodesic polyline.
    s : float
        Normalized coordinate in ``[0, 1]``.

    Returns
    -------
    np.ndarray
        Point on the geodesic as a 1D vector of shape (3,).
    """
    return np.asarray(
        _impl.point_from_geodesic(geodesic._geodesic_impl, float(s))  # type: ignore[attr-defined]
    ).reshape(
        -1
    )  # type: ignore[attr-defined]


def geodesic_resample(
    geodesic: "Geodesic",
    coordinates: ArrayLike,
) -> NDArray[np.float64]:
    """Resample a geodesic at normalized coordinates.

    Parameters
    ----------
    geodesic : Geodesic
        Geodesic polyline.
    coordinates : array_like
        1D array of normalized coordinates in ``[0, 1]``.

    Returns
    -------
    np.ndarray
        Resampled points of shape ``(T, 3)``.
    """
    coords = np.asarray(coordinates, dtype=np.float64)
    if coords.ndim != 1:
        raise ValueError("coordinates must be a 1D array.")
    return np.asarray(
        _impl.geodesic_resample_matrix(  # type: ignore[attr-defined]
            geodesic._geodesic_impl,  # type: ignore[attr-defined]
            coords,
        )
    )


def parallel_transport(
    tangent_vector: "TangentVector",
    point: "Point",
) -> "TangentVector":
    """Parallel transport a tangent vector to a destination point.

    Parameters
    ----------
    tangent_vector : TangentVector
        Tangent vector to transport.
    point : Point
        Destination point.

    Returns
    -------
    TangentVector
        Transported tangent vector.
    """
    from .tangent_vector import TangentVector

    tv_impl = _impl.parallel_transport(
        tangent_vector._tv_impl,  # type: ignore[attr-defined]
        point._point_impl,  # type: ignore[attr-defined]
    )
    return TangentVector.from_impl(tv_impl)  # type: ignore[attr-defined]


def logarithmic_map(point: "Point", target: "Point") -> "TangentVector":
    """Compute the logarithmic map of ``target`` at ``point``.

    Parameters
    ----------
    point : Point
        Base point on the mesh.
    target : Point
        Target point on the mesh.

    Returns
    -------
    TangentVector
        Tangent vector at ``point`` pointing toward ``target``.
    """
    from .tangent_vector import TangentVector

    tv_impl = _impl.logarithmic_map(
        point._point_impl,  # type: ignore[attr-defined]
        target._point_impl,  # type: ignore[attr-defined]
    )
    return TangentVector.from_impl(tv_impl)  # type: ignore[attr-defined]


def exponential_map(tangent_vector: "TangentVector") -> "Point":
    """Apply the exponential map of a tangent vector.

    Parameters
    ----------
    tangent_vector : TangentVector
        Tangent vector to apply.

    Returns
    -------
    Point
        Point reached by the exponential map.
    """
    from .point import Point

    pt_impl = _impl.exponential_map(tangent_vector._tv_impl)  # type: ignore[attr-defined]
    return Point(pt_impl)


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


def projx(mesh: Mesh, xs: np.ndarray) -> np.ndarray:
    """
    Inputs:
        mesh: mesh on which doing computation
        xs : (N, 3) Tensors on the mesh. Points to be projected on the mesh
    Outputs:
        ys : (N, 3) Points projected on the mesh
    """
    return _impl.projx(mesh._mesh_impl, xs)

def validate_projx(mesh: Mesh, xs: np.ndarray, ps: np.ndarray):
    _impl.validate_projx(mesh._mesh_impl, xs, ps)

def num_points_on_mesh(mesh: Mesh, xs: np.ndarray) -> int:
    return _impl.num_points_on_mesh(mesh._mesh_impl, xs)


def proju(mesh: Mesh, xs: np.ndarray, vs: np.ndarray) -> np.ndarray:
    """
    Inputs:
        mesh: mesh on which vectors shall be projected
        xs : (N, 3) Tensors on the mesh. Application points of the tangent vector
        vs : (N, 3) Tensors on the mesh. The tangent vector at each xs.
    Outputs:
        ys : (N, 3) Vectors projected in the tangent space
    """
    return _impl.proj(mesh._mesh_impl, xs, vs)


def proj_transformation(mesh: Mesh, xs: np.ndarray, vs: np.ndarray) -> np.ndarray:
    """
    Inputs:
        mesh: mesh on which vectors shall be projected
        xs : (N, 3) Tensors on the mesh. Application points of the tangent vector
        vs : (N, 3) Tensors on the mesh. The tangent vector at each xs.
    Outputs:
        tf : (N, 3, 3) Vectors projected in the tangent space
    """
    N = xs.shape[0]
    impl_res = _impl.proj_transformation(mesh._mesh_impl, xs, vs)
    res = np.zeros((N, 3, 3))
    for i in range(N):
        res[i, :, :] = impl_res[i]
    return res


def closest_face_normal_and_vertex(
    mesh: Mesh, xs: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Inputs:
        mesh: mesh on which vectors shall be projected
        xs : (N, 3) Tensors on the mesh. Application points of the tangent vector
    """
    res = _impl.closest_face_normal_and_vertex(mesh._mesh_impl, xs)
    return np.array(res[0]), np.array(res[1])
