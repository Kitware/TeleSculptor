// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_FEATUREOPTIONS_H_
#define TELESCULPTOR_FEATUREOPTIONS_H_

#include "PointOptions.h"

class vtkMaptkFeatureTrackRepresentation;

class FeatureOptionsPrivate;

class FeatureOptions : public PointOptions
{
  Q_OBJECT

public:
  explicit FeatureOptions(
    vtkMaptkFeatureTrackRepresentation*, QString const& settingsGroup,
    QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~FeatureOptions() override;

public slots:
  void setFeaturesWithDescVisible(bool);
  void setFeaturesWithoutDescVisible(bool);

protected slots:
  void setTrailsWithDescVisible(bool);
  void setTrailsWithoutDescVisible(bool);
  void setTrailsLength(int);
  void setTrailsStyle(int);

private:
  QTE_DECLARE_PRIVATE_RPTR(FeatureOptions)
  QTE_DECLARE_PRIVATE(FeatureOptions)

  QTE_DISABLE_COPY(FeatureOptions)
};

#endif
