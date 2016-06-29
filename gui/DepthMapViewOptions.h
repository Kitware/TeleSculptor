#ifndef DEPTHMAPVIEWOPTIONS_H
#define DEPTHMAPVIEWOPTIONS_H

#include <qtGlobal.h>

#include <QtGui/QWidget>

//class vtkActor;

class DepthMapViewOptionsPrivate;

class DepthMapViewOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapViewOptions(const QString &settingsGroup,
                           QWidget *parent = 0, Qt::WindowFlags flags = 0);
  virtual ~DepthMapViewOptions();

//  void addActor(vtkActor* actor);

//  bool isPointsChecked();
//  bool isSurfacesChecked();

//  void enablePoints();
//  void enableSurfaces();

signals:
//  void modified();
//  void depthMapChanged();

protected slots:
//  void switchPointsVisible(bool);
//  void switchSurfacesVisible(bool);

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapViewOptions)
  QTE_DECLARE_PRIVATE(DepthMapViewOptions)

  QTE_DISABLE_COPY(DepthMapViewOptions)
};

#endif // DEPTHMAPVIEWOPTIONS_H
