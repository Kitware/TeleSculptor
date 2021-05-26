// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DATAARRAYS_H_
#define TELESCULPTOR_DATAARRAYS_H_

namespace LandmarkArrays
{
  static char const* const TrueColor = "truecolor";
  static char const* const Elevation = "elevation";
  static char const* const Observations = "observations";
}

namespace DepthMapArrays
{
  // CAUTION: These must match the names in the VTI files
  static char const* const Depth = "Depths";
  static char const* const TrueColor = "Color";
  static char const* const Weight = "Weight";
  static char const* const Uncertainty = "Uncertainty";
}

#endif
