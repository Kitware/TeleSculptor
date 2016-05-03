#ifndef DEPTHMAPVIEWOPTIONS_H
#define DEPTHMAPVIEWOPTIONS_H

#include <qtGlobal.h>

#include <QtGui/QWidget>

#include <QButtonGroup>
#include <QFormLayout>

class vtkImageActor;

class DepthMapViewOptionsPrivate;

class DepthMapViewOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapViewOptions(const QString &settingsGroup,
                           QWidget *parent = 0, Qt::WindowFlags flags = 0);
  virtual ~DepthMapViewOptions();

  void addActor(vtkImageActor* actor);

  void cleanModes();

signals:
  void modified();

protected slots:
  void switchDisplayMode(bool checked);

private:
  QButtonGroup* bGroup;
  QFormLayout* layout;

  void addDepthMapMode(std::string name, bool needGradient);

  QTE_DECLARE_PRIVATE_RPTR(DepthMapViewOptions)
  QTE_DECLARE_PRIVATE(DepthMapViewOptions)

  QTE_DISABLE_COPY(DepthMapViewOptions)
};

#endif // DEPTHMAPVIEWOPTIONS_H
