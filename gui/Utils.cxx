// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

//-----------------------------------------------------------------------------
#ifndef __APPLE__
double GetDoubleClickInterval()
{
#ifdef __WIN32__
  return GetDoubleClickTime();
#endif
  // Default for X11 platforms
  return 400;
}
#endif
