#ifndef DEPTHMAPOPTIONS_H
#define DEPTHMAPOPTIONS_H

#include <qtGlobal.h>

#include <QtGui/QWidget>

class vtkProp3D;

class DepthMapOptionsPrivate;

class DepthMapOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapOptions(const QString &settingsGroup,
                           QWidget *parent = 0, Qt::WindowFlags flags = 0);
  virtual ~DepthMapOptions();

  void addActor(std::string type, vtkProp3D *actor);

  bool isPointsChecked();
  bool isSurfacesChecked();
  bool isVerticesChecked();

  void enableDM(std::string type);
//  void enablePoints();
//  void enableSurfaces();
//  void enableVertices();

signals:
  void modified();
  void depthMapChanged();

protected slots:
//  void switchPointsVisible(bool);
//  void switchSurfacesVisible(bool);
  void switchVisible(bool);

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapOptions)
  QTE_DECLARE_PRIVATE(DepthMapOptions)

  QTE_DISABLE_COPY(DepthMapOptions)
};

#endif // DEPTHMAPOPTIONS_H
