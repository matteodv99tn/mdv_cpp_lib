project = "Matteo's C++ Library"
copyright = 'Matteo Dalle Vedove - 2024-2025'
author = 'Matteo Dalle Vedove'
release = 'v0.0'

extensions = [
    "sphinx_copybutton",
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
]

source_suffix = ['.rst', '.md']

pygments_style = 'sphinx'

html_theme = 'furo'
autosummary_generate = True
