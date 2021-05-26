// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include <AppKit/AppKit.h>

//-----------------------------------------------------------------------------
#ifdef __APPLE__
double GetDoubleClickInterval()
{
  // Convert to milliseconds
  return 1000 * [NSEvent doubleClickInterval];
}
#endif

