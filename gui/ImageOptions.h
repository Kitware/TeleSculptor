// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_IMAGEOPTIONS_H_
#define TELESCULPTOR_IMAGEOPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class vtkImageActor;

class ImageOptionsPrivate;

class ImageOptions : public QWidget
{
  Q_OBJECT

public:
  explicit ImageOptions(QString const& settingsGroup,
                        QWidget* parent = nullptr,
                        Qt::WindowFlags flags = {});
  ~ImageOptions() override;

  void addActor(vtkImageActor*);

signals:
  void modified();

protected slots:
  void setOpacity(double);

private:
  QTE_DECLARE_PRIVATE_RPTR(ImageOptions)
  QTE_DECLARE_PRIVATE(ImageOptions)

  QTE_DISABLE_COPY(ImageOptions)
};

#endif
