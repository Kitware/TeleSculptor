#include "DepthMapOptions.h"
#include "ui_DepthMapOptions.h"

DepthMapOptions::DepthMapOptions(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::DepthMapOptions)
{
  ui->setupUi(this);
}

DepthMapOptions::~DepthMapOptions()
{
  delete ui;
}
