# Python bindings tests

This folder contains pytest-based tests for the SWIG Python bindings.

Recommended workflow (from repo root):

```bash
uv venv .venv -p 3.12
uv pip install -e .
uv pip install pytest
uv run pytest bindings/tests
```
