#ifndef VOLUMEOPTIONS_H_
#define VOLUMEOPTIONS_H_

#include <qtGlobal.h>

#include <QtGui/QWidget>

class vtkActor;

class VolumeOptionsPrivate;

class VolumeOptions : public QWidget
{
  Q_OBJECT

public:
  explicit VolumeOptions(const QString &settingsGroup,
      QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~VolumeOptions();

  void setActor(vtkActor* actor);

public slots:
  void showColorizeSurfaceMenu(bool state);
  void updateColorizeSurfaceMenu(QString text);

private:
  QTE_DECLARE_PRIVATE_RPTR(VolumeOptions)
  QTE_DECLARE_PRIVATE(VolumeOptions)

  QTE_DISABLE_COPY(VolumeOptions)
};

#endif
