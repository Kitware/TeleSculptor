############################################
             MAP-Tk Plugins
############################################

This directory contains MAP-Tk plugin scripts.  These scripts are not plugins
for use within MAP-Tk.  Rather they are plugins for third-party software to
allow the third-party software to load in files produced by MAP-Tk.

=================== ===========================================================
`Blender Plugins`_  These are plugins for the open source Blender_ 3D creation
                    suite.  These plugins allow Blender to read and write
                    cameras files in the KRTD file format used by MAP-Tk.
                    Blender plugins are written in Python and are called
                    "Add-ons".  Instructions on how to install Blender Add-ons
                    can be found `here
                    <https://docs.blender.org/manual/en/dev/preferences/addons.html>`_.

`SketchUp Plugins`_ These are plugins for the SketchUp_ 3D creation suite.
                    These plugins allow SketchUp to import a MAP-Tk config file
                    which loads images and cameras into "Photo Match" views.
                    This allows a SketchUp user to draw 3D models on top of the
                    images using camera calibration from MAP-Tk.  SketchUp
                    plugins are written in Ruby.  To install SketchUp plugins
                    follow the directions in this
                    `article <https://help.sketchup.com/en/article/38583>`_ or 
					simply copy the folder and top-level file into the 'Plugins'
					directory in your SketchUp installation.
=================== ===========================================================


.. Appendix I: References
.. ======================

.. _Blender Plugins: blender/
.. _Blender: https://www.blender.org/
.. _SketchUp Plugins: sketchup/
.. _SketchUp: https://www.sketchup.com/
