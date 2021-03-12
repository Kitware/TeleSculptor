TeleSculptor Flatpaking
=======================

This Flatpak manifest file is work in progress, it still does not build TeleSculptor successfully.
 
Flatpak is a utility for software deployment and package management for Linux. It offers a sandbox environment in which users can run application software in isolation from the rest of the system.

Building and running com.github.Kitware.TeleSculptor
----------------------------------------------------

1) Install flatpak-builder
   - Ubuntu, Debian, sudo apt install flatpak-builder
   - Arch, sudo pacman -Syu flatpak-builder
   - CentOS, Fedora, dnf install flatpak-builder
   - openSUSE, sudo zypper in flatpak-builder
2) git clone git://github.com/flathub/shared-modules.git
3) Download com.github.Kitware.TeleSculptor.yaml
4) flatpak-builder --install --user build com.github.Kitware.TeleSculptor.yaml --force-clean
5) Wait until finish and run with, flatpak run com.github.Kitware.TeleSculptor
