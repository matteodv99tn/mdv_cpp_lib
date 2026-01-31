from __future__ import annotations

from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from .kernel import MeshKernel
from .point import Point
from .vertex import Vertex

class InexactMeshKernel(MeshKernel):
    def __init__(self, mesh: "Mesh") -> None: ...
    def distance_matrix(
        self,
        points1: Sequence[Point | Vertex],
        points2: Sequence[Point | Vertex] | None = ...,
    ) -> NDArray[np.float64]: ...
    def set_points1(self, points1: Sequence[Point | Vertex]) -> None: ...
    def find_pointset_max_lengthscale(
        self,
        points: Sequence[Point | Vertex],
        num_steps: int = ...,
    ) -> float: ...
    def __call__(
        self,
        points: Sequence[Point | Vertex],
        points2: Sequence[Point | Vertex] | None = ...,
        lengthscale: float = ...,
    ) -> NDArray[np.float64]: ...

from .mesh import Mesh
