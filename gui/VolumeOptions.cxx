/*ckwg +29
 * Copyright 2016-2018 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
  VolumeOptionsPrivate() {}

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

  connect(d->UI.doubleSpinBoxOcclusionThreshold, &QDoubleSpinBox::editingFinished,
          this, &VolumeOptions::updateOcclusionThreshold);
  connect(d->UI.checkBoxRemoveOcclusion,
          &QCheckBox::stateChanged,
          this, &VolumeOptions::removeOcclusionChanged);
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
void VolumeOptions::setCameras(kwiver::vital::camera_map_sptr cameras)
{
  QTE_D();

  d->colorizeSurfaceOptions->setCameras(cameras);
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr VolumeOptions::getCameras() const
{
  QTE_D();

  return d->colorizeSurfaceOptions->getCameras();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setVideoConfig(std::string const& videoPath,
                                   kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->colorizeSurfaceOptions->setVideoConfig(videoPath, config);
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
void VolumeOptions::colorize()
{
  QTE_D();

  d->colorizeSurfaceOptions->colorize();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setCurrentFrame(int frame)
{
  QTE_D();

  d->colorizeSurfaceOptions->setCurrentFrame(frame);
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
  d->UI.doubleSpinBoxOcclusionThreshold->setEnabled(state);
  d->UI.checkBoxRemoveOcclusion->setEnabled(state);
  d->colorizeSurfaceOptions->enableMenu(state);
  if (state)
  {
    this->colorize();
  }
  d->volumeActor->GetMapper()->SetScalarVisibility(state);
  emit modified();
}

//-----------------------------------------------------------------------------
void VolumeOptions::updateColorizeSurfaceMenu(QString const& text)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setText(text);

  emit modified();
}

//-----------------------------------------------------------------------------
void VolumeOptions::updateOcclusionThreshold()
{
  QTE_D();

  d->colorizeSurfaceOptions->setOcclusionThreshold(d->UI.doubleSpinBoxOcclusionThreshold->value());
  d->colorizeSurfaceOptions->forceColorize();
}

//-----------------------------------------------------------------------------
void VolumeOptions::removeOcclusionChanged(int removeOcclusion)
{
  QTE_D();
  d->colorizeSurfaceOptions->setRemoveOcclusion(removeOcclusion);
  d->colorizeSurfaceOptions->forceColorize();
}
