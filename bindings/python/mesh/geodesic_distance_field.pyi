from collections.abc import Sequence
from typing import overload

import numpy as np
from numpy.typing import NDArray

from .mesh import Mesh
from .point import Point
from .vertex import Vertex

class GeodesicDistanceField:
    def __init__(self, mesh: Mesh, sources: Sequence[Point | Vertex]) -> None: ...
    @overload
    def __call__(self, query: Point | Vertex) -> float: ...
    @overload
    def __call__(self, query: Sequence[Point | Vertex]) -> NDArray[np.float64]: ...
