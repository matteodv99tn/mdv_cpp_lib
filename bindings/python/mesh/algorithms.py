import numpy as np

from . import _mesh_impl as _impl
from . import Geodesic, Mesh


def length(geodesic: Geodesic) -> float:
    return _impl.length(geodesic._geodesic_impl)


def solve_path(mesh: Mesh, x0: np.ndarray, x1: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Inputs:
        mesh: mesh on which doing computation
        x0 : (N, 3) Tensors on the mesh. The starting point.
        x1 : (N, 3) Tensors on the mesh. The end point.
        t: (T,) Tensor of time values between 0 and 1 (inclusive).
        projx: Bool. If true, projects x onto the mesh after every step.
    Outputs:
        xt : (T, N, 3) Tensors on the path.
        ut : (T, N, 3) Tensors on the tangent plane of xt. The vector field at xt that transports from x0 to x1.
    """
    assert x0.shape == x1.shape
    assert len(x0.shape) == 2
    assert len(t.shape) == 1
    N = x0.shape[0]
    T = t.shape[0]

    xt = np.zeros((T, N, 3))
    ut = np.zeros((T, N, 3))
    
    path_data = _impl.solve_path(mesh._mesh_impl, x0, x1, t)

    for i in range(N):
        xt[:, i, :] = path_data[i][0]
        ut[:, i, :] = path_data[i][0]

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
