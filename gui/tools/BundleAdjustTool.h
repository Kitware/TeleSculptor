// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_BUNDLEADJUSTTOOL_H_
#define TELESCULPTOR_BUNDLEADJUSTTOOL_H_

#include "AbstractTool.h"

class BundleAdjustToolPrivate;

class BundleAdjustTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit BundleAdjustTool(QObject* parent = nullptr);
  ~BundleAdjustTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

  bool callback_handler(camera_map_sptr cameras, landmark_map_sptr landmarks);

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(BundleAdjustTool)
  QTE_DECLARE_PRIVATE(BundleAdjustTool)
  QTE_DISABLE_COPY(BundleAdjustTool)
};

#endif
