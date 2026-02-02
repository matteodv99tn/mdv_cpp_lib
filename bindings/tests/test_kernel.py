from __future__ import annotations

import numpy as np

from mdv.mesh import MeshKernel, Point  # type: ignore[import-not-found]


def test_distance_matrix_shape_and_symmetry(cube_mesh) -> None:
    kernel = MeshKernel(cube_mesh)
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    v0, v1, v2 = faces[0]
    points = [
        Point.from_cartesian(cube_mesh, vertices[v0]),
        Point.from_cartesian(cube_mesh, vertices[v1]),
        Point.from_cartesian(cube_mesh, vertices[v2]),
    ]

    distances = kernel.distance_matrix(points)
    assert distances.shape == (3, 3)
    assert np.allclose(distances, distances.T, atol=1e-8)
    assert np.all(distances >= 0.0)

    assert kernel.is_positive_definite(np.eye(3))
