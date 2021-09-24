// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_SAVEKEYFRAMETOOL_H_
#define TELESCULPTOR_SAVEKEYFRAMETOOL_H_

#include "AbstractTool.h"

class SaveKeyFrameToolPrivate;

class SaveKeyFrameTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit SaveKeyFrameTool(QObject* parent = nullptr);
  ~SaveKeyFrameTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(SaveKeyFrameTool)
  QTE_DECLARE_PRIVATE(SaveKeyFrameTool)
  QTE_DISABLE_COPY(SaveKeyFrameTool)
};

#endif
