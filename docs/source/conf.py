project = 'Panda_Trajectory'
copyright = '2025, IIT-RAIN'
author = 'IIT-RAIN'
release = '1'

# Python-specific extensions (NOT Breathe!)
extensions = [
    "sphinx.ext.autodoc",      # Auto-import Python modules
    "sphinx.ext.napoleon",     # Support docstring formats
    "sphinx.ext.viewcode",     # Link to source code
]

templates_path = ['_templates']
exclude_patterns = []
language = 'English'

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_css_files = ['custom.css']
html_theme_options = {
    "collapse_navigation": False,
    "sticky_navigation": True,
    "prev_next_buttons_location": "both",
    "navigation_depth": -1
}

# IMPORTANT: Add path so Sphinx can import ros2_py_pkg
import sys
import os
sys.path.insert(0, os.path.abspath('../..'))
