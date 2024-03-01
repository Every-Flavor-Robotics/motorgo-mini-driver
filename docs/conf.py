# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'MotorGo Mini Driver'
copyright = '2023, Every Flavor Robotics LLC, MotorGo LLC'
author = 'Every Flavor Robotics LLC, MotorGo LLC'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'breathe',
    'exhale',
    'sphinx_rtd_theme',
    'sphinx_multiversion'
]

breathe_projects = {
    'MotorGo Mini Driver': './doxyoutput/xml',  # Point to your Doxygen XML output directory
}

breathe_default_project = 'MotorGo Mini Driver'

# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "api_reference.rst",
    "afterTitleDescription": "MotorGo Mini Driver API",
    "doxygenStripFromPath":  "..",
    # Heavily encouraged optional argument (see docs)
    "rootFileTitle":         "API Reference",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin":    "INPUT = ../include",
    "fullApiSubSectionTitle": "API Reference",
    "fullToctreeMaxDepth": 2
}


templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_sidebars = {'**': ['logo-text.html', 'globaltoc.html', 'searchbox.html']}


# -- Options for Multiversion  ------------------------------------------------

# Whitelist the versions you want included (adjust as needed)
smv_branch_whitelist = r'^main|dev|api-docs$'
smv_tag_whitelist = r'^.*$'

# Output subdirectory based on branch for simple version separation
smv_outputdir_format = '{ref.name}'

# Customize the version selection layout
smv_tag_dropdown = True
smv_rename_assets_path = False