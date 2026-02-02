"""Mesh bindings.

Public, typed wrappers around the SWIG-generated mesh bindings.
"""

from .algorithms import (  # type: ignore[attr-defined]
    exponential_map,
    geodesic_resample,
    length,
    logarithmic_map,
    parallel_transport,
    point_from_geodesic,
    solve_path,
)
from .tangent_vector import TangentVector
from .face import Face
from .flat_parameterisation import FlatParameterisation
from .geodesic import Geodesic
from .gaussian_process import InexactGaussianProcess
from .inexact_kernel import InexactMeshKernel
from .kernel import MeshKernel
from .mesh import Mesh
from .point import Point
from .vertex import Vertex

__all__ = [
    "Face",
    "FlatParameterisation",
    "Geodesic",
    "InexactGaussianProcess",
    "InexactMeshKernel",
    "Mesh",
    "MeshKernel",
    "Point",
    "TangentVector",
    "Vertex",
    "length",
    "logarithmic_map",
    "parallel_transport",
    "point_from_geodesic",
    "geodesic_resample",
    "exponential_map",
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
