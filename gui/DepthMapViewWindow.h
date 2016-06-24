#ifndef DEPTHMAPVIEWWINDOW_H
#define DEPTHMAPVIEWWINDOW_H

#include <QMainWindow>

namespace Ui {
class DepthMapViewWindow;
}

class DepthMapViewWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit DepthMapViewWindow(QWidget *parent = 0);
  ~DepthMapViewWindow();

private:
  Ui::DepthMapViewWindow *ui;
};

#endif // DEPTHMAPVIEWWINDOW_H
