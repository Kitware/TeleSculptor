// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_RUNALLTOOL_H_
#define TELESCULPTOR_RUNALLTOOL_H_

#include "AbstractTool.h"

class RunAllToolPrivate;

class RunAllTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit RunAllTool(QObject* parent = nullptr);
  ~RunAllTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

public slots:
  void cancel() override;

protected:
  void run() override;
  bool runTool(AbstractTool* tool, bool save_final = true);
  void saveResults(AbstractTool* tool);

protected slots:
  void forwardInterimResults(std::shared_ptr<ToolData> data);
  void reportToolError(QString const& msg);

private:
  QTE_DECLARE_PRIVATE_RPTR(RunAllTool)
  QTE_DECLARE_PRIVATE(RunAllTool)
  QTE_DISABLE_COPY(RunAllTool)
};

#endif
