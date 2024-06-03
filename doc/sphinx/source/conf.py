# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'OpenLoong Dynamics Control'
copyright = '2024, Authorname'
author = 'Authorname'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.imgmath', 'sphinx.ext.todo', 'breathe',   # doxygen surrport
              'recommonmark',           # markdown support
              'sphinx_markdown_tables'] # markdown table support

source_parsers = {
    '.md': 'recommonmark.parser.CommonMarkParser',
}

source_suffix = {
 '.rst': 'restructuredtext',
 '.txt': 'restructuredtext',
 '.md': 'markdown',
}

# Breathe Configuration
breathe_projects = {"OpenLoong Dynamics Control": "../../doxygen/xml/"}
breathe_default_project = "OpenLoong Dynamics Control"

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
