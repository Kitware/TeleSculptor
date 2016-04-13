#include "DepthMapView.h"
#include "ui_DepthMapView.h"

DepthMapView::DepthMapView(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::DepthMapView)
{
  ui->setupUi(this);
}

DepthMapView::~DepthMapView()
{
  delete ui;
}
