# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

version: 2


import os
import sys
import datetime
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

# Set the current date
today = datetime.date.today()
current_date = today.strftime("%B %d, %Y")  # Format: 'October 03, 2024'


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
html_show_sphinx = False


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


version = '0.0.1'

html_title = 'Indra RL Lab Documentation'


html_theme_options = {
    'analytics_id': 'G-XXXXXXXXXX',  #  Provided by Google in your dashboard
    'analytics_anonymize_ip': False,
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    'vcs_pageview_mode': '',
    'style_nav_header_background': '#004254',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# If true, Sphinx will generate the general index (genindex).
html_use_index = True

# If true, Sphinx will generate a module index (modindex).
html_domain_indices = True

html_js_files = [
    '_static/flyout.js',
]


html_context = {
  'display_github': True,
  'github_user': 'javi-carrera',
  'github_repo': 'Indra-RL-Lab',
  'github_version': 'documentation',
  'conf_py_path': '/docs/',
  'current_date': current_date, 
  'footer_template': '_templates/footer.html',
  'flyingmenu_template': '_templates/flyingmenu.html',
}

# Tell Sphinx to include your custom footer in the layout
html_sidebars = {
    '**': ['globaltoc.html', 'relations.html', 'sourcelink.html', 'searchbox.html', 'footer.html'],
}

html_show_copyright = False
html_favicon = '_static/img/logomini.png'


# Add path to static files
html_static_path = ['_static']
templates_path = ['_templates']


# -- Options for formats ---------------------------------------------
# -- Options for LaTeX output ---------------------------------------------
author = (
    'Indra RL Lab Team'
)

latex_documents = [
    (master_doc, 'YourProject.tex', 'Indra RL Lab Documentation', author,
     'manual', True),
]