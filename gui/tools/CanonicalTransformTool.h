// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_CANONICALTRANSFORMTOOL_H_
#define TELESCULPTOR_CANONICALTRANSFORMTOOL_H_

#include "AbstractTool.h"

class CanonicalTransformToolPrivate;

class CanonicalTransformTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit CanonicalTransformTool(QObject* parent = nullptr);
  ~CanonicalTransformTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return false; }

  bool execute(QWidget* window = nullptr) override;

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(CanonicalTransformTool)
  QTE_DECLARE_PRIVATE(CanonicalTransformTool)
  QTE_DISABLE_COPY(CanonicalTransformTool)
};

#endif
