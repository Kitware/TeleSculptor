#include "DepthMapFilterOptions.h"
#include "ui_DepthMapFilterOptions.h"

DepthMapFilterOptions::DepthMapFilterOptions(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::DepthMapFilterOptions)
{
  ui->setupUi(this);
}

DepthMapFilterOptions::~DepthMapFilterOptions()
{
  delete ui;
}
