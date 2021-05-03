// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_EDITMODE_H_
#define TELESCULPTOR_EDITMODE_H_

#include <QMetaType>

enum class EditMode
{
  None,
  GroundControlPoints,
  CameraRegistrationPoints,
};

Q_DECLARE_METATYPE(EditMode)

#endif
