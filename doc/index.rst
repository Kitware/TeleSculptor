.. TeleSculptor documentation master file, created by
   sphinx-quickstart on Wed Mar 31 23:56:53 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. include:: manuals/version.rst

TeleSculptor
============

.. image:: /../gui/icons/256x256/telesculptor.png
   :align: right

Introduction
------------

Welcome to TeleSculptor.  This software is free and open source software developed by Kitware under multiple SBIR contracts for DARPA and AFRL.  The purpose of the software is to 
calibrate cameras and extract 3D information from video and images, especially aerial video.  The software is designed to have different workflows that can either improve the manual 
modelling process in SketchUp or provide fully automatic 3D reconstruction.

TeleSculptor v |version| |space| marks the second official release of the software. There are several improvements to the application interface and algorithm robustness since the previous v1.0 
release.  Please refer to the release notes file for details on all the changes.  While TeleSculptor continues to become more broadly useful across imaging scenarios, it is still 
optimized for the use case of aerial video flying a 360-degree orbit around a target object.  TeleSculptor will give best results on a video clip containing one orbit around an 
object.  We will improve robustness on more general imaging scenarios in future releases.

TeleSculptor is a cross-platform desktop application for Windows, Linux, and Mac.  This document uses the Windows version of the application to illustrate usage, but everything should 
work nearly the same on other platforms.

Branding: Origins and Destinations
----------------------------------

TeleSculptor has its origins in a software project developed as “MAP-Tk”, the Motion-imagery Aerial Photogrammetry Toolkit.  The original software was not an end user application but 
a collection of developer tools and libraries.  As the software evolved it developed a graphical application that we now call TeleSculptor.  At the same time, the software libraries 
of MAP-Tk were reorganized into a new, broader toolkit called KWIVER (Kitware Image and Video Exploitation and Retrieval).  So, the original MAP-Tk has dissolved away leaving behind a 
TeleSculptor application powered by KWIVER.  However, some uses of the name MAP-Tk may persist for historical reasons.

Getting Started
---------------

* For instructions on how to install Telesculptor, see :ref:`Installation <installation>`.

* To start a new Telesculptor project, see :ref:`Processing Steps <processingsteps>`.

.. toctree::
   :maxdepth: 2
   :hidden:

   manuals/installation
   manuals/views
   manuals/workflows
   manuals/processingsteps
   manuals/advancedtools
   manuals/computeoptions
   manuals/advancedconfig
   manuals/cameracalibration
