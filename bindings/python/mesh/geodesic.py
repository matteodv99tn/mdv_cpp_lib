from __future__ import annotations

from typing import List

import numpy as np
from numpy.typing import NDArray

try:
    from ._mesh_impl import Geodesic as _GeodesicImpl  # type: ignore[import-not-found]
except ImportError:  # pragma: no cover - generated at build time

    class _GeodesicImpl:  # type: ignore[no-redef]
        def __len__(self) -> int:
            raise RuntimeError("SWIG bindings are not built yet.")

        def __getitem__(self, _idx: int) -> NDArray[np.float64]:
            raise RuntimeError("SWIG bindings are not built yet.")


class Geodesic:
    """Python wrapper for a mesh geodesic."""

    def __init__(self, geodesic_impl: _GeodesicImpl):
        self._geodesic_impl = geodesic_impl

    def length(self) -> float:
        """Return the length of the geodesic.

        Returns
        -------
        float
            Geodesic length.
        """
        from ._mesh_impl import length  # type: ignore[import-not-found]

        return length(self._geodesic_impl)

    def to_list(self) -> List[NDArray[np.float64]]:
        """Return the polyline as a list of 3D points.

        Returns
        -------
        list[np.ndarray]
            List of 3D points representing the geodesic path.
        """
        result = []
        for i in range(len(self._geodesic_impl)):
            result.append(self._geodesic_impl[i])
        return result
