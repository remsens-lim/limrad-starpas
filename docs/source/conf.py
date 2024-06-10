# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
import starpas
# sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('../src/'))

# -- Project information -----------------------------------------------------

project = 'LIMRAD-STARPAS'
copyright = '2024, Jonas Witthuhn'
author = 'Jonas Witthuhn'

# The full version, including alpha/beta/rc tags
release = starpas.__version__


# -- General configuration ---------------------------------------------------
source_suffix = {
    '.rst': 'restructuredtext',
    # '.md': 'markdown',
    '.md': 'myst-nb',
    '.ipynb': 'myst-nb',
}

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    # 'myst_parser',
    'myst_nb',
    'sphinx.ext.coverage',
    'sphinx.ext.napoleon',
    'sphinx.ext.graphviz',
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.extlinks',
]

nb_execution_mode = "off"

extlinks = {
    "doi": ("https://doi.org/%s", "doi:"),
}

graphviz_output_format = "svg"

intersphinx_mapping = {
    'python': ('https://docs.python.org/', None),
    'pandas': ('https://pandas.pydata.org/pandas-docs/stable', None),
    'xarray': ('https://xarray.pydata.org/en/stable', None),
    'numpy': ('https://numpy.org/doc/stable', None),
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

napoleon_type_aliases = {
    # general terms
    "sequence": ":term:`sequence`",
    "iterable": ":term:`iterable`",
    "callable": ":py:func:`callable`",
    "dict_like": ":term:`dict-like <mapping>`",
    "dict-like": ":term:`dict-like <mapping>`",
    "path-like": ":term:`path-like <path-like object>`",
    "mapping": ":term:`mapping`",
    "file-like": ":term:`file-like <file-like object>`",
    # special terms
    # "same type as caller": "*same type as caller*",  # does not work, yet
    # "same type as values": "*same type as values*",  # does not work, yet
    "timedelta": "~datetime.timedelta",
    "string": ":class:`string <str>`",
    # numpy terms
    "array_like": ":term:`array_like`",
    "array_like": ":term:`array_like <array_like>`",
}


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'alabaster'

html_theme_options = {
    # "fixed_sidebar": "true",
    "logo": "lim-remsens-logo.png",
    "logo_name": "true",
    "description": "University of Leipzig, Institute for Meteorology"
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']