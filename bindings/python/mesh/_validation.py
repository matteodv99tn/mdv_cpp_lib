from __future__ import annotations

from typing import Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray


def as_vector(
    value: ArrayLike,
    *,
    size: int | None = None,
    name: str = "value",
    dtype: np.dtype | type = np.float64,
) -> NDArray[np.float64]:
    """Convert input to a 1D numpy vector and validate its size.

    Parameters
    ----------
    value : array_like
        Input data to convert.
    size : int, optional
        Required vector length.
    name : str, default="value"
        Name used in error messages.
    dtype : numpy dtype, default=np.float64
        Desired dtype for the returned array.

    Returns
    -------
    np.ndarray
        1D numpy array.

    Raises
    ------
    ValueError
        If the input cannot be interpreted as a vector of the expected size.
    """
    arr = np.asarray(value, dtype=dtype)
    if arr.ndim == 2 and 1 in arr.shape:
        arr = arr.reshape(-1)
    if arr.ndim != 1:
        raise ValueError(f"{name} must be a 1D vector; got shape {arr.shape}.")
    if size is not None and arr.size != size:
        raise ValueError(f"{name} must have length {size}; got length {arr.size}.")
    return arr


def as_matrix(
    value: ArrayLike,
    *,
    shape: tuple[int, int] | None = None,
    name: str = "value",
    dtype: np.dtype | type = np.float64,
) -> NDArray[np.float64]:
    """Convert input to a 2D numpy array and validate its shape.

    Parameters
    ----------
    value : array_like
        Input data to convert.
    shape : tuple[int, int], optional
        Required matrix shape. Use ``-1`` for a free dimension.
    name : str, default="value"
        Name used in error messages.
    dtype : numpy dtype, default=np.float64
        Desired dtype for the returned array.

    Returns
    -------
    np.ndarray
        2D numpy array.

    Raises
    ------
    ValueError
        If the input cannot be interpreted as a 2D array of the expected shape.
    """
    arr = np.asarray(value, dtype=dtype)
    if arr.ndim != 2:
        raise ValueError(f"{name} must be a 2D array; got shape {arr.shape}.")
    if shape is not None:
        exp_rows, exp_cols = shape
        if exp_rows != -1 and arr.shape[0] != exp_rows:
            raise ValueError(f"{name} must have {exp_rows} rows; got {arr.shape[0]}.")
        if exp_cols != -1 and arr.shape[1] != exp_cols:
            raise ValueError(
                f"{name} must have {exp_cols} columns; got {arr.shape[1]}."
            )
    return arr
