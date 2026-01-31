from __future__ import annotations

from mdv.mesh import FlatParameterisation, Point  # type: ignore[import-not-found]


def test_flat_parameterisation_roundtrip(cube_mesh) -> None:
    param = FlatParameterisation(cube_mesh)
    point = Point.from_vertex(cube_mesh.vertex(0))

    uv = param.project(point)
    assert uv.shape == (2,)

    recovered = param.retrieve(uv)
    assert recovered.position.shape == (3,)

    assert isinstance(param.is_inside_mesh(uv), bool)
    assert param.min_uv().shape == (2,)
    assert param.max_uv().shape == (2,)
