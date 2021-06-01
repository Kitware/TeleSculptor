// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_FIELDINFORMATION_H_
#define TELESCULPTOR_FIELDINFORMATION_H_

#include <QtCore/QByteArray>

struct FieldInformation
{
  QByteArray name;
  double totalRange[2];
  double autoRange[2];
};

#endif
