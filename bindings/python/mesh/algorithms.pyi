from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .geodesic import Geodesic
from .mesh import Mesh
from .point import Point
from .tangent_vector import TangentVector

def length(geodesic: Geodesic) -> float: ...
def point_from_geodesic(geodesic: Geodesic, s: float) -> NDArray[np.float64]: ...
def geodesic_resample(
    geodesic: Geodesic,
    coordinates: ArrayLike,
) -> NDArray[np.float64]: ...
def parallel_transport(
    tangent_vector: TangentVector,
    point: Point,
) -> TangentVector: ...
def logarithmic_map(point: Point, target: Point) -> TangentVector: ...
def exponential_map(tangent_vector: TangentVector) -> Point: ...
def solve_path(
    mesh: Mesh,
    x0: ArrayLike,
    x1: ArrayLike,
    t: ArrayLike,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...
