from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .point import Point
from .vertex import Vertex

class MeshKernel:
    def __init__(self, mesh: "Mesh") -> None: ...
    def distance_matrix(
        self,
        points1: Sequence[Point | Vertex],
        points2: Optional[Sequence[Point | Vertex]] = ...,
    ) -> NDArray[np.float64]: ...
    def __call__(
        self,
        points1: Sequence[Point | Vertex],
        points2: Sequence[Point | Vertex],
        lengthscale: float = ...,
    ) -> NDArray[np.float64]: ...
    @staticmethod
    def evaluate_squared_exponential(
        distance_matrix: ArrayLike,
        lengthscale: float = ...,
    ) -> NDArray[np.float64]: ...
    def find_pointset_max_lengthscale(
        self,
        points: Sequence[Point | Vertex],
        num_steps: int = ...,
    ) -> float: ...
    @staticmethod
    def find_matrix_max_lengthscale(
        distance_matrix: ArrayLike,
        num_iters: int = ...,
        initial_lengthscale: float = ...,
    ) -> float: ...

from .mesh import Mesh
