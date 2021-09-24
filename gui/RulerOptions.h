// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_RULEROPTIONS_H_
#define TELESCULPTOR_RULEROPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class RulerHelper;
class RulerOptionsPrivate;

class RulerOptions : public QWidget
{
  Q_OBJECT

public:
  explicit RulerOptions(QString const& settingsGroup,
                        QWidget* parent = nullptr,
                        Qt::WindowFlags flags = {});
  ~RulerOptions() override;

  void setRulerHelper(RulerHelper* helper);

signals:
  void modified();
  void resetRuler();

private:
  QTE_DECLARE_PRIVATE_RPTR(RulerOptions);
  QTE_DECLARE_PRIVATE(RulerOptions);
  QTE_DISABLE_COPY(RulerOptions);
};

#endif
