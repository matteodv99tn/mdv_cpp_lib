from . import _mesh_impl as _impl
from . import Geodesic

def length(geodesic: Geodesic) -> float:
    return _impl.length(geodesic._geodesic_impl)
