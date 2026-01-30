from .mesh import Mesh
from .vertex import Vertex
from .face import Face
from .point import Point
from .geodesic import Geodesic
from .kernel import MeshKernel
from .inexact_kernel import InexactMeshKernel
from .flat_parameterisation import FlatParameterisation
from .algorithms import length

# Export the main classes
__all__ = [
    'Mesh',
    'Vertex',
    'Face',
    'Point',
    'Geodesic',
    'MeshKernel',
    'InexactMeshKernel',
    'FlatParameterisation',
    'load_from_file',
    'mesh_directory',
    'length',
]

def mesh_directory() -> str:
    from . import _mesh_impl as _impl
    return _impl.mesh_directory()
