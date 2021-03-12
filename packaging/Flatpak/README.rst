TeleSculptor Flatpak
====================

This Flatpak manifest file is work in progress, it still does not build TeleSculptor successfully.
 
Flatpak is a utility for software deployment and package management for Linux. It offers a sandbox environment in which users can run application software in isolation from the rest of the system.

Building and running com.github.Kitware.TeleSculptor
----------------------------------------------------

#. Create new working folder.
#. Install flatpak-builder, for Ubuntu, ``sudo apt install flatpak-builder``
#. Install KDE Sdk, choose version 5.15 (system), ``flatpak install org.kde.Platform org.kde.Sdk``
#. Clone sumodules, ``git clone git://github.com/flathub/shared-modules.git``
#. Download com.github.Kitware.TeleSculptor.yaml
#. ``flatpak-builder --install --user build com.github.Kitware.TeleSculptor.yaml --force-clean``
#. Wait until finish and run with, ``flatpak run com.github.Kitware.TeleSculptor``
