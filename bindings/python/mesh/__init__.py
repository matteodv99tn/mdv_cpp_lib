"""Mesh bindings.

Public, typed wrappers around the SWIG-generated mesh bindings.
"""

from .algorithms import length, solve_path
from .face import Face
from .flat_parameterisation import FlatParameterisation
from .geodesic import Geodesic
from .inexact_kernel import InexactMeshKernel
from .kernel import MeshKernel
from .mesh import Mesh
from .point import Point
from .vertex import Vertex

__all__ = [
    "Face",
    "FlatParameterisation",
    "Geodesic",
    "InexactMeshKernel",
    "Mesh",
    "MeshKernel",
    "Point",
    "Vertex",
    "length",
    "mesh_directory",
    "solve_path",
]


def mesh_directory() -> str:
    """Return the default mesh data directory.

    Returns
    -------
    str
        Absolute path to the mesh data directory.
    """
    from . import _mesh_impl as _impl  # type: ignore[import-not-found]

    return _impl.mesh_directory()
