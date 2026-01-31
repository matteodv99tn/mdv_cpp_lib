def test_import_mesh_module() -> None:
    import mdv.mesh as mesh  # type: ignore[import-not-found]

    assert hasattr(mesh, "Mesh")
