#include "DepthMapViewWindow.h"
#include "ui_DepthMapViewWindow.h"

DepthMapViewWindow::DepthMapViewWindow(QWidget* parent)
  : QMainWindow(parent), ui(new Ui::DepthMapViewWindow)
{
  ui->setupUi(this);
}

DepthMapViewWindow::~DepthMapViewWindow()
{
  delete ui;
}
