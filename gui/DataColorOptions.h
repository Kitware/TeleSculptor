// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_DATACOLOROPTIONS_H_
#define TELESCULPTOR_DATACOLOROPTIONS_H_

#include <qtGlobal.h>
#include <qtMath.h>

#include <QIcon>
#include <QWidget>

class vtkScalarsToColors;

class DataColorOptionsPrivate;

class DataColorOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DataColorOptions(QString const& settingsGroup,
                            QWidget* parent = nullptr,
                            Qt::WindowFlags flags = {});
  ~DataColorOptions() override;

  QIcon icon() const;

  double minimum() const;
  double maximum() const;

  vtkScalarsToColors* scalarsToColors() const;

public slots:
  void setAvailableRange(double lower, double upper,
                         double autoLower = -qInf(),
                         double autoUpper = +qInf());
  void setGradient(int);

signals:
  void modified();
  void iconChanged(QIcon);

protected slots:
  void updateMinimum();
  void updateMaximum();

private:
  QTE_DECLARE_PRIVATE_RPTR(DataColorOptions)
  QTE_DECLARE_PRIVATE(DataColorOptions)

  QTE_DISABLE_COPY(DataColorOptions)
};

#endif
