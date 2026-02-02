from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .mesh import Mesh
from .point import Point

class InexactGaussianProcess:
    def __init__(
        self, mesh: Mesh, lengthscale: float, sigma_noise: float = ...
    ) -> None: ...
    def train(self, points: list[Point], y_ref: ArrayLike) -> None: ...
    def predict(self, points: list[Point]) -> NDArray[np.float64]: ...
