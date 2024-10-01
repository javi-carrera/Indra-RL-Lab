# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

version: 2

import os
import sys
sys.path.insert(0, os.path.abspath('../../Docker/')) #equivalent to adding src files to PYTHONPATH
sys.path.insert(0, os.path.abspath('../../Unity/')) #equivalent to adding src files to PYTHONPATH

try:
    import sphinxcontrib.spelling  # noqa: F401

    enable_spell_check = True
except ImportError:
    enable_spell_check = False

# Try to enable copy button
try:
    import sphinx_copybutton  # noqa: F401

    enable_copy_button = True
except ImportError:
    enable_copy_button = False


# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Indra RL Lab'
copyright = '2024, Javier Carrera, Guillermo Escolano, Nicolás Gastón, Rodrigo Sánchez, Sebastián Laiseca, María López-Chaves'
author = 'Javier Carrera, Guillermo Escolano, Nicolás Gastón, Rodrigo Sánchez, Sebastián Laiseca, María López-Chaves'
release = '0.0.0'


# conf.py

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

toctree_depth = 4

extensions = [
    'sphinx.ext.autodoc',
    "sphinx.ext.autosummary",
    "sphinx.ext.ifconfig",
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    "myst_parser", #for writing .md files
    'sphinx.ext.intersphinx',
    'sphinxcontrib.youtube',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# The master toctree document.
master_doc = "index"

autodoc_typehints = "description" #ocumenting acceptable argument types and return value types of functions

if enable_spell_check:
    extensions.append("sphinxcontrib.spelling")

if enable_copy_button:
    extensions.append("sphinx_copybutton")

exclude_patterns = []

language = "en"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'

html_last_updated_fmt = "%b %d, %Y"

templates_path = ['_templates']

# Add these to customize the sidebar links
# Define the sidebar configuration
html_sidebars = {
    "**": [
        "sidebar.html",  # Include your custom sidebar
        'globaltoc.html',
        'relations.html',
        'searchbox.html',
    ],
}

html_static_path = ['_static']
html_css_files = [
    'css/custom.css',
]
html_logo = "_static/img/logo.png"


version = '1.0'


# If true, Sphinx will generate the general index (genindex).
html_use_index = True

# If true, Sphinx will generate a module index (modindex).
html_domain_indices = True

html_js_files = [
    'https://cdnjs.cloudflare.com/ajax/libs/jquery/3.5.1/jquery.min.js', 
    '_static/flyout.js'
]


html_context = {
  'display_github': True,
  'github_user': 'javi-carrera',
  'github_repo': 'Indra-RL-Lab',
  'github_version': 'documentation',
  'conf_py_path': '/docs/',
}


html_show_copyright = False
html_favicon = '_static/img/logomini.png'




# -- Options for formats ---------------------------------------------
# -- Options for LaTeX output ---------------------------------------------
latex_elements = {
    'papersize': 'a4paper',  # You can change to 'letterpaper'
    'pointsize': '10pt',     # Default font size
    'preamble': '',          # Add any custom LaTeX preamble if needed
}

latex_documents = [
    ('index', 'MyProject.tex', 'My Project Documentation', 'Author Name', 'manual'),
]
