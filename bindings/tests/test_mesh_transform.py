from __future__ import annotations

import numpy as np

from mdv.mesh import Mesh  # type: ignore[import-not-found]


def test_mesh_transform(cube_mesh: Mesh) -> None:
    transform = np.eye(4, dtype=np.float64)
    cube_mesh.transform(transform)
