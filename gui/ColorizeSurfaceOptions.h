#ifndef COLORIZESURFACEOPTIONS_H
#define COLORIZESURFACEOPTIONS_H

#include <qtGlobal.h>

#include <QtGui/QWidget>

class ColorizeSurfaceOptionsPrivate;

class ColorizeSurfaceOptions : public QWidget
{
  Q_OBJECT

public:
  explicit ColorizeSurfaceOptions(const QString &settingsGroup,
      QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~ColorizeSurfaceOptions();

  void addColorDisplay(std::string name);

signals:
  void colorModeChanged(QString);

public slots:
  void toggleAllFramesMenu();

private:
  QTE_DECLARE_PRIVATE_RPTR(ColorizeSurfaceOptions)
  QTE_DECLARE_PRIVATE(ColorizeSurfaceOptions)

  QTE_DISABLE_COPY(ColorizeSurfaceOptions)
};

#endif // COLORIZESURFACEOPTIONS_H
