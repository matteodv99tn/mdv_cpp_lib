from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

try:
    from ._mesh_impl import (  # type: ignore[import-not-found]
        InexactGaussianProcess as _InexactGaussianProcessImpl,
        PointVector,
        create_inexact_gaussian_process,
    )
except ImportError:  # pragma: no cover - generated at build time

    class _InexactGaussianProcessImpl:  # type: ignore[no-redef]
        def __init__(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def train(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

        def predict(self, *_args, **_kwargs):
            raise RuntimeError("SWIG bindings are not built yet.")

    class PointVector(list):  # type: ignore[no-redef]
        pass

    def create_inexact_gaussian_process(*_args, **_kwargs):
        raise RuntimeError("SWIG bindings are not built yet.")


from .mesh import Mesh
from .point import Point


class InexactGaussianProcess:
    """Gaussian process regression on a mesh using geodesic kernels.

    Parameters
    ----------
    mesh : Mesh
        Surface mesh.
    lengthscale : float
        Kernel lengthscale.
    sigma_noise : float, default=0.0
        Observation noise standard deviation.
    """

    def __init__(self, mesh: Mesh, lengthscale: float, sigma_noise: float = 0.0):
        self._gp_impl = create_inexact_gaussian_process(
            mesh._mesh_impl,  # type: ignore[attr-defined]
            float(lengthscale),
            float(sigma_noise),
        )

    def train(self, points: list[Point], y_ref: ArrayLike) -> None:
        """Train the GP on input points and target values.

        Parameters
        ----------
        points : list[Point]
            Training points on the mesh.
        y_ref : array_like, shape (N, D)
            Target values aligned with ``points``.
        """
        y_mat = np.asarray(y_ref, dtype=np.float64)
        if y_mat.ndim != 2:
            raise ValueError("y_ref must be a 2D matrix.")
        pt_impls = [pt._point_impl for pt in points]  # type: ignore[attr-defined]
        self._gp_impl.train(PointVector(pt_impls), y_mat)

    def predict(self, points: list[Point]) -> NDArray[np.float64]:
        """Predict output values at query points.

        Parameters
        ----------
        points : list[Point]
            Query points on the mesh.

        Returns
        -------
        np.ndarray
            Predicted values.
        """
        pt_impls = [pt._point_impl for pt in points]  # type: ignore[attr-defined]
        return np.asarray(self._gp_impl.predict(PointVector(pt_impls)))
