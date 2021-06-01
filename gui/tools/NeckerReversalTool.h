// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_NECKERREVERSALTOOL_H_
#define TELESCULPTOR_NECKERREVERSALTOOL_H_

#include "AbstractTool.h"

class NeckerReversalTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit NeckerReversalTool(QObject* parent = nullptr);
  ~NeckerReversalTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return false; }

  bool execute(QWidget* window = nullptr) override;

protected:
  void run() override;

private:
  QTE_DISABLE_COPY(NeckerReversalTool)
};

#endif
