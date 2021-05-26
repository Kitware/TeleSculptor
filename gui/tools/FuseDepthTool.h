// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_FUSEDEPTHTOOL_H_
#define TELESCULPTOR_FUSEDEPTHTOOL_H_

#include "AbstractTool.h"

class FuseDepthToolPrivate;

class FuseDepthTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit FuseDepthTool(QObject* parent = nullptr);
  virtual ~FuseDepthTool();

  virtual Outputs outputs() const override;

  /// Get if the tool can be canceled.
  virtual bool isCancelable() const override { return true; }

  virtual bool execute(QWidget* window = nullptr) override;

protected:
  virtual void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(FuseDepthTool)
  QTE_DECLARE_PRIVATE(FuseDepthTool)
  QTE_DISABLE_COPY(FuseDepthTool)
};

#endif
