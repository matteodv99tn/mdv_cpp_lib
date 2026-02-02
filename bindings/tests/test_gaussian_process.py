from __future__ import annotations

import numpy as np

from mdv.mesh import InexactGaussianProcess, Point  # type: ignore[import-not-found]


def test_inexact_gaussian_process_train_predict(cube_mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    v0, v1, v2 = faces[0]
    points = [
        Point.from_cartesian(cube_mesh, vertices[v0]),
        Point.from_cartesian(cube_mesh, vertices[v1]),
        Point.from_cartesian(cube_mesh, vertices[v2]),
    ]
    y_ref = np.array([[0.1], [0.2], [0.3]], dtype=np.float64)

    gp = InexactGaussianProcess(cube_mesh, lengthscale=1.0, sigma_noise=0.0)
    gp.train(points, y_ref)

    preds = gp.predict(points)
    assert preds.shape == y_ref.shape
