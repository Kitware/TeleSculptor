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

#include "ColorizeSurfaceOptions.h"
#include "ui_ColorizeSurfaceOptions.h"

#include "tools/MeshColoration.h"

#include <vtkActor.h>

#include <vtkLookupTable.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

#include <qtStlUtil.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QDebug>

//-----------------------------------------------------------------------------
class ColorizeSurfaceOptionsPrivate
{
public:
  ColorizeSurfaceOptionsPrivate(): volumeActor(nullptr), currentFrame(-1)
  {}

  Ui::ColorizeSurfaceOptions UI;
  qtUiState uiState;

  vtkActor* volumeActor;

  kwiver::vital::config_block_sptr videoConfig;
  std::string videoPath;
  kwiver::vital::camera_map_sptr cameras;

  QString krtdFile;
  QString frameFile;

  int currentFrame;
};

QTE_IMPLEMENT_D_FUNC(ColorizeSurfaceOptions)

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::ColorizeSurfaceOptions(
  const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new ColorizeSurfaceOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.radioButtonCurrentFrame, &QAbstractButton::clicked,
          this, &ColorizeSurfaceOptions::currentFrameSelected);

  connect(d->UI.radioButtonAllFrames, &QAbstractButton::clicked,
          this, &ColorizeSurfaceOptions::allFrameSelected);


  connect(d->UI.buttonCompute, &QAbstractButton::clicked,
          this, &ColorizeSurfaceOptions::colorize);

  connect(d->UI.comboBoxColorDisplay,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ColorizeSurfaceOptions::changeColorDisplay);

  connect(d->UI.doubleSpinBoxOcclusionThreshold, &QDoubleSpinBox::editingFinished,
          this, &ColorizeSurfaceOptions::updateOcclusionThreshold);
  connect(d->UI.checkBoxRemoveOcclusion,
          &QCheckBox::stateChanged,
          this, &ColorizeSurfaceOptions::removeOcclusionChanged);

  d->krtdFile = QString();
  d->frameFile = QString();

  d->UI.comboBoxColorDisplay->setDuplicatesEnabled(false);
  this->OcclusionThreshold = 1;
  this->RemoveOcclusion = true;
  this->InsideColorize = false;
  this->LastColorizedFrame = INVALID_FRAME;
}

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::~ColorizeSurfaceOptions()
{
  QTE_D();

  d->uiState.save();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::initFrameSampling(int nbFrames)
{
  QTE_D();

  d->UI.spinBoxFrameSampling->setMaximum(nbFrames-1);
  d->UI.spinBoxFrameSampling->setValue((nbFrames-1)/20);
}

//-----------------------------------------------------------------------------
int ColorizeSurfaceOptions::getFrameSampling() const
{
  QTE_D();

  return d->UI.spinBoxFrameSampling->value();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setCurrentFrame(int frame)
{
  QTE_D();

  if (d->currentFrame != frame)
  {
    d->currentFrame = frame;

    if (d->UI.radioButtonCurrentFrame->isChecked()
        && d->UI.radioButtonCurrentFrame->isEnabled())
    {
      currentFrameSelected();
    }
  }
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setActor(vtkActor* actor)
{
  QTE_D();

  d->volumeActor = actor;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setVideoConfig(std::string const& path,
                                            kwiver::vital::config_block_sptr config)
{
  QTE_D();

  d->videoConfig = config;
  d->videoPath = path;
}

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr ColorizeSurfaceOptions::getVideoConfig() const
{
  QTE_D();

  return d->videoConfig;
}

//-----------------------------------------------------------------------------
std::string ColorizeSurfaceOptions::getVideoPath() const
{
  QTE_D();

  return d->videoPath;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setCameras(kwiver::vital::camera_map_sptr cameras)
{
  QTE_D();

  d->cameras = cameras;
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr ColorizeSurfaceOptions::getCameras() const
{
  QTE_D();

  return d->cameras;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::enableMenu(bool state)
{
  QTE_D();

  d->UI.radioButtonAllFrames->setEnabled(state);
  d->UI.radioButtonCurrentFrame->setEnabled(state);
  d->UI.doubleSpinBoxOcclusionThreshold->setEnabled(state);
  d->UI.checkBoxRemoveOcclusion->setEnabled(state);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::changeColorDisplay()
{
  QTE_D();

  vtkPolyData* volume = vtkPolyData::SafeDownCast(
    d->volumeActor->GetMapper()->GetInput());
  const char* activeScalar = qPrintable(d->UI.comboBoxColorDisplay->currentText());
  volume->GetPointData()->SetActiveScalars(activeScalar);

  vtkMapper* mapper = d->volumeActor->GetMapper();

  if(volume->GetPointData()->GetScalars() &&
     volume->GetPointData()->GetScalars()->GetNumberOfComponents() != 3)
  {
    vtkNew<vtkLookupTable> table;
    table->SetRange(volume->GetPointData()->GetScalars()->GetRange());
    table->Build();
    mapper->SetLookupTable(table.Get());
    mapper->SetColorModeToMapScalars();
    mapper->UseLookupTableScalarRangeOn();
  }
  else
  {
    mapper->SetColorModeToDirectScalars();
    mapper->CreateDefaultLookupTable();
    mapper->UseLookupTableScalarRangeOff();
  }

  mapper->Update();

  emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::forceColorize()
{
  this->LastColorizedFrame = INVALID_FRAME;
  colorize();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::colorize()
{
  QTE_D();
  if (! this->InsideColorize)
  {
    this->InsideColorize = true;
    int colorizedFrame = (d->UI.radioButtonCurrentFrame->isChecked()) ? d->currentFrame : -1;
    while (this->LastColorizedFrame != colorizedFrame)
    {
      this->LastColorizedFrame = colorizedFrame;

      if (d->cameras->size() == 0)
      {
        d->UI.comboBoxColorDisplay->setEnabled(true);
        emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
        return;
      }
      QEventLoop loop;
      vtkPolyData* volume = vtkPolyData::SafeDownCast(d->volumeActor->GetMapper()
                                                      ->GetInput());
      MeshColoration* coloration =
        new MeshColoration(d->videoConfig, d->videoPath, d->cameras);

      coloration->SetInput(volume);
      coloration->SetOutput(volume);
      coloration->SetFrameSampling(d->UI.spinBoxFrameSampling->value());
      coloration->SetOcclusionThreshold(this->OcclusionThreshold);
      coloration->SetRemoveOcclusion(this->RemoveOcclusion);
      coloration->SetFrame(this->LastColorizedFrame);
      coloration->SetAverageColor(true);
      connect(coloration, &MeshColoration::resultReady,
              this, &ColorizeSurfaceOptions::meshColorationHandleResult);
      connect( coloration, &MeshColoration::resultReady, &loop, &QEventLoop::quit );
      connect(coloration, &MeshColoration::finished,
              coloration, &MeshColoration::deleteLater);
      coloration->start();
      loop.exec();
      colorizedFrame = (d->UI.radioButtonCurrentFrame->isChecked()) ? d->currentFrame : -1;
    }
    this->InsideColorize = false;
  }
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::meshColorationHandleResult(MeshColoration* coloration)
{
  QTE_D();
  if (coloration)
  {
    vtkPolyData* volume = coloration->GetOutput();
    volume->GetPointData()->SetActiveScalars("MeanColoration");
    d->UI.comboBoxColorDisplay->setCurrentIndex(
      d->UI.comboBoxColorDisplay->findText("MeanColoration"));
  }
  d->UI.comboBoxColorDisplay->setEnabled(true);

  emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
}



//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::enableAllFramesParameters(bool state)
{
  QTE_D();

  d->UI.buttonCompute->setEnabled(state);
  d->UI.spinBoxFrameSampling->setEnabled(state);
  d->UI.comboBoxColorDisplay->setEnabled(false);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::allFrameSelected()
{
  enableAllFramesParameters(true);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::currentFrameSelected()
{
  QTE_D();

  enableAllFramesParameters(false);

  if (d->currentFrame != -1)
  {
    colorize();
  }
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::updateOcclusionThreshold()
{
  QTE_D();
  this->setOcclusionThreshold(d->UI.doubleSpinBoxOcclusionThreshold->value());
  this->forceColorize();
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::removeOcclusionChanged(int removeOcclusion)
{
  this->setRemoveOcclusion(removeOcclusion);
  this->forceColorize();
}
