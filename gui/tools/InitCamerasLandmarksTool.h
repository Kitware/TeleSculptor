// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_INITCAMERASLANDMARKSTOOL_H_
#define TELESCULPTOR_INITCAMERASLANDMARKSTOOL_H_

#include "AbstractTool.h"

class InitCamerasLandmarksToolPrivate;

class InitCamerasLandmarksTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit InitCamerasLandmarksTool(QObject* parent = nullptr);
  ~InitCamerasLandmarksTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

  bool callback_handler(camera_map_sptr cameras, landmark_map_sptr landmarks,
                        feature_track_set_changes_sptr track_changes);

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(InitCamerasLandmarksTool)
  QTE_DECLARE_PRIVATE(InitCamerasLandmarksTool)
  QTE_DISABLE_COPY(InitCamerasLandmarksTool)
};

#endif
