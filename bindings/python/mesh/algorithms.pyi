from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .geodesic import Geodesic
from .mesh import Mesh

def length(geodesic: Geodesic) -> float: ...
def solve_path(
    mesh: Mesh,
    x0: ArrayLike,
    x1: ArrayLike,
    t: ArrayLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...
