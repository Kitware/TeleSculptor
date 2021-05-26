// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_TRACKFEATURESTOOL_H_
#define TELESCULPTOR_TRACKFEATURESTOOL_H_

#include "AbstractTool.h"

class TrackFeaturesToolPrivate;

class TrackFeaturesTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit TrackFeaturesTool(QObject* parent = nullptr);
  ~TrackFeaturesTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(TrackFeaturesTool)
  QTE_DECLARE_PRIVATE(TrackFeaturesTool)
  QTE_DISABLE_COPY(TrackFeaturesTool)
};

#endif
