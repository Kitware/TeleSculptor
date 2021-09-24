// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_SAVEFRAMETOOL_H_
#define TELESCULPTOR_SAVEFRAMETOOL_H_

#include "AbstractTool.h"

class SaveFrameToolPrivate;

class SaveFrameTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit SaveFrameTool(QObject* parent = nullptr);
  ~SaveFrameTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(SaveFrameTool)
  QTE_DECLARE_PRIVATE(SaveFrameTool)
  QTE_DISABLE_COPY(SaveFrameTool)
};

#endif
