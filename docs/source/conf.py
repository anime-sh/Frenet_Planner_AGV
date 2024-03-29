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
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
sys.path.append(os.path.abspath('./frenet_planner_agv'))

# -- Project information -----------------------------------------------------

project = 'Frenet Planner AGV'
copyright = '2021,AGV'
author = 'AGV Planning Module'

# The full version, including alpha/beta/rc tags
release = '0.0.1'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx_rtd_theme',
    'sphinx.ext.intersphinx',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages'
]

autoclass_content = 'both'

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

# set master document
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
# html_theme = 'classic'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = []


# -- Build code documentation ------------------------------------------------
def run_apidoc(_):
    from sphinx.ext.apidoc import main
    import os
    import sys
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

    # specify path where the source files should be placed
    source_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "frenet_planner_agv")

    # specify path of the module to be documented
    module_dir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "..", "..", "frenet_planner_agv")

    # call apidoc main
    main(['-e', '-o', source_dir, module_dir, '--force'])


def setup(app):
    app.connect('builder-inited', run_apidoc)


# EOF
