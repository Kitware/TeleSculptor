# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os.path

from docutils import nodes, utils
from docutils.parsers.rst.states import Struct

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'TeleSculptor'
version = '1.2.0'
copyright = '2021, Kitware, Inc.'
author = 'Kitware, Inc.'

kwiver_version = '1.6.0'

repo_root = 'https://github.com/Kitware/TeleSculptor'
relnotes_root = f'{repo_root}/blob/master/doc/release-notes'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [
    'Thumbs.db',
    '.DS_Store',
    'replacements.rst',
]

rst_epilog = f'''
.. include:: replacements.rst

.. _Release Notes: {relnotes_root}/{version}.txt

.. |kwiver_version| replace:: {kwiver_version}
'''


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# The logo to use on the doc page
html_logo = "images/TeleSculptor_logo_small.png"

html_favicon = "../gui/icons/telesculptor.ico"

html_theme_options = {
    'logo_only': True,
}

html_css_files = [
    'css/custom.css',
]

# The master toctree document
master_doc = 'index'


# -- Customizations ----------------------------------------------------------

icon_root = '../gui/icons'
icon_sizes = [22, 16]
srcdir = ''

def locate_icon(name, size):
    for d in [f'{size}x{size}@2', f'{size}x{size}']:
        p = os.path.join(icon_root, d, f'{name}.png')
        print(p)
        if os.path.isfile(os.path.join(srcdir, p)):
            return p

    return None

def build_icon(rawtext, text, lineno, inliner, options={}, sizes=icon_sizes):
    if text in ['-', 'blank']:
        return None

    options['alt'] = utils.unescape(text)
    for s in sizes:
        uri = locate_icon(text, s)
        print(s, uri)
        if uri is not None:
            options['uri'] = uri
            options['classes'] = [f'icon-{s}']
            return nodes.image(rawtext, **options)

    return None

def action_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    parts = text.split()
    print(parts)

    # Build icon element
    icon = build_icon(rawtext, parts[0], lineno, inliner, sizes=[16, 22])

    # Create node
    options['classes'] = ['action']
    node = nodes.inline(rawtext, '', **options)

    if icon is not None:
        node += icon
        node += nodes.Text(' ', ' ')

    title = ' '.join(parts[1:])
    node += nodes.Text(title, title)

    # Return resulting node
    return [node], []

def make_parsed_text_role(class_names=[], node_class=nodes.inline):
    def parsed_text_role(name, rawtext, text, lineno, inliner,
                         options={}, content=[]):
        # Prepare context for nested parsing
        memo = Struct(document=inliner.document,
                      reporter=inliner.reporter,
                      language=inliner.language)

        # Create parent node
        options['classes'] = class_names
        parent = node_class(rawtext, '', **options)

        # Parse role text for markup and add to parent
        processed, messages = inliner.parse(text, lineno, memo, parent)
        parent += processed

        # Return parent node, and any messages from nested parsing
        return [parent], messages

    return parsed_text_role

def setup(app):
    global srcdir
    srcdir = app.srcdir

    app.add_role('var', make_parsed_text_role(class_names=['math', 'script']))
    app.add_role('math', make_parsed_text_role(class_names=['math']))
    app.add_role('path', make_parsed_text_role(class_names=['filepath']))
    app.add_role('menu', make_parsed_text_role(class_names=['menu']))
    app.add_role('shortcut', make_parsed_text_role(class_names=['shortcut']))

    app.add_role('action', action_role)
