// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "VolumeOptions.h"
#include "ui_VolumeOptions.h"

#include "ColorizeSurfaceOptions.h"
#include "WorldView.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QMenu>
#include <QToolButton>
#include <QWidgetAction>

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>

#include <string>

//-----------------------------------------------------------------------------
class VolumeOptionsPrivate
{
public:
  VolumeOptionsPrivate():colorizeSurfaceOptions(nullptr),volumeActor(nullptr)
  {}

  void setPopup(QToolButton* button, QWidget* popup);

  Ui::VolumeOptions UI;
  qtUiState uiState;
  ColorizeSurfaceOptions* colorizeSurfaceOptions;

  vtkActor* volumeActor;

};

QTE_IMPLEMENT_D_FUNC(VolumeOptions)

//-----------------------------------------------------------------------------
void VolumeOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//-----------------------------------------------------------------------------
VolumeOptions::VolumeOptions(const QString &settingsGroup, QWidget* parent,
                             Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new VolumeOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  d->colorizeSurfaceOptions = new ColorizeSurfaceOptions(settingsGroup, this);
  d->setPopup(d->UI.toolButtonColorizeSurfaceMenu, d->colorizeSurfaceOptions);

  // Connect signals/slots
  connect(d->UI.checkBoxColorizeSurface, &QAbstractButton::toggled,
          this, &VolumeOptions::showColorizeSurfaceMenu);

  connect(d->colorizeSurfaceOptions, &ColorizeSurfaceOptions::colorModeChanged,
          this, &VolumeOptions::updateColorizeSurfaceMenu);

  connect(d->UI.doubleSpinBoxSurfaceThreshold,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          qobject_cast<WorldView*>(parent), &WorldView::computeContour);
}

//-----------------------------------------------------------------------------
VolumeOptions::~VolumeOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setActor(vtkActor *actor)
{
  QTE_D();

  d->volumeActor = actor;
  d->colorizeSurfaceOptions->setActor(actor);
}

//-----------------------------------------------------------------------------
void VolumeOptions::initFrameSampling(int nbFrames)
{
  QTE_D();

  d->colorizeSurfaceOptions->initFrameSampling(nbFrames);
}

//-----------------------------------------------------------------------------
int VolumeOptions::getFrameSampling() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getFrameSampling();
}

//-----------------------------------------------------------------------------
double VolumeOptions::getOcclusionThreshold() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getOcclusionThreshold();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setCamera(
  kwiver::vital::frame_id_t id, kwiver::vital::camera_sptr const& camera)
{
  QTE_D();

  d->colorizeSurfaceOptions->setCamera(id, camera);
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr VolumeOptions::getCameras() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getCameras();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setVideoConfig(std::string const& path,
                                   kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->colorizeSurfaceOptions->setVideoConfig(path, config);
}

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr VolumeOptions::getVideoConfig() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getVideoConfig();
}

//-----------------------------------------------------------------------------
std::string VolumeOptions::getVideoPath() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getVideoPath();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setMaskConfig(std::string const& path,
                                   kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->colorizeSurfaceOptions->setMaskConfig(path, config);
}

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr VolumeOptions::getMaskConfig() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getMaskConfig();
}

//-----------------------------------------------------------------------------
std::string VolumeOptions::getMaskPath() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getMaskPath();
}

//-----------------------------------------------------------------------------
void VolumeOptions::colorize()
{
  QTE_D();

  d->colorizeSurfaceOptions->colorize();
}

//-----------------------------------------------------------------------------
void VolumeOptions::forceColorize()
{
  QTE_D();

  d->colorizeSurfaceOptions->forceColorize();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setCurrentFrame(kwiver::vital::frame_id_t frame)
{
  QTE_D();

  d->colorizeSurfaceOptions->setCurrentFrame(frame);
}

//-----------------------------------------------------------------------------
void VolumeOptions::setSurfaceColored(bool enabled)
{
  QTE_D();

  d->UI.checkBoxColorizeSurface->setChecked(enabled);
}

//-----------------------------------------------------------------------------
bool VolumeOptions::isColorOptionsEnabled()
{
  QTE_D();

  return d->UI.checkBoxColorizeSurface->isCheckable()
         && d->UI.checkBoxColorizeSurface->isChecked();
}

//-----------------------------------------------------------------------------
void VolumeOptions::showColorizeSurfaceMenu(bool state)
{
  QTE_D();
  d->UI.toolButtonColorizeSurfaceMenu->setEnabled(state);
  d->colorizeSurfaceOptions->enableMenu(state);
  if (state)
  {
    this->forceColorize();
  }
  d->volumeActor->GetMapper()->SetScalarVisibility(state);
  emit modified();
}

//-----------------------------------------------------------------------------
void VolumeOptions::reshowColorizeSurfaceMenu()
{
  QTE_D();
  if (d->volumeActor)
  {
    d->volumeActor->GetMapper()->Update();
    this->showColorizeSurfaceMenu(d->UI.checkBoxColorizeSurface->isChecked());
  }
}

//-----------------------------------------------------------------------------
void VolumeOptions::updateColorizeSurfaceMenu(QString const& text)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setText(text);

  emit modified();
}
