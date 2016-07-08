#ifndef MAPTK_DEPTHMAPVIEWWINDOW_H_
#define MAPTK_DEPTHMAPVIEWWINDOW_H_

#include <QMainWindow>

namespace Ui
{
class DepthMapViewWindow;
}

class DepthMapViewWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit DepthMapViewWindow(QWidget* parent = 0);
  ~DepthMapViewWindow();

private:
  Ui::DepthMapViewWindow* ui;
};

#endif
