from __future__ import annotations

import numpy as np

from mdv.mesh import MeshKernel, Point  # type: ignore[import-not-found]


def test_distance_matrix_shape_and_symmetry(cube_mesh) -> None:
    kernel = MeshKernel(cube_mesh)
    points = [
        Point.from_vertex(cube_mesh.vertex(0)),
        Point.from_vertex(cube_mesh.vertex(1)),
        Point.from_vertex(cube_mesh.vertex(2)),
    ]

    distances = kernel.distance_matrix(points)
    assert distances.shape == (3, 3)
    assert np.allclose(distances, distances.T, atol=1e-8)
    assert np.all(distances >= 0.0)
