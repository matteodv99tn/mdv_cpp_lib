from __future__ import annotations

import numpy as np

from mdv.mesh import Mesh  # type: ignore[import-not-found]


def test_mesh_matrices_shapes(cube_mesh: Mesh) -> None:
    vertices = cube_mesh.get_vertex_matrix()
    faces = cube_mesh.get_face_matrix()

    assert vertices.ndim == 2
    assert vertices.shape[1] == 3
    assert faces.ndim == 2
    assert faces.shape[1] == 3
    assert faces.dtype == np.int32

def test_vertex_and_face_vectors_are_1d(cube_mesh: Mesh) -> None:
    vertex = cube_mesh.vertex(0)
    face = cube_mesh.face(0)

    v_pos = vertex.position
    v_norm = vertex.normal
    f_norm = face.normal

    assert v_pos.shape == (3,)
    assert v_norm.shape == (3,)
    assert f_norm.shape == (3,)

    random_face = cube_mesh.random_face()
    assert random_face.id >= 0 and random_face.id < cube_mesh.num_faces
