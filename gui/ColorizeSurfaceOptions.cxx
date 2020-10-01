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

  d->krtdFile = QString();
  d->frameFile = QString();

  d->UI.comboBoxColorDisplay->setDuplicatesEnabled(false);
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
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::changeColorDisplay()
{
  QTE_D();

  vtkPolyData* volume = vtkPolyData::SafeDownCast(
    d->volumeActor->GetMapper()->GetInput());

  volume->GetPointData()->SetActiveScalars(
    qPrintable(d->UI.comboBoxColorDisplay->currentText()));

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
void ColorizeSurfaceOptions::colorize()
{
  QTE_D();

  if (d->cameras->size() > 0)
  {
    d->UI.comboBoxColorDisplay->clear();

    vtkPolyData* volume = vtkPolyData::SafeDownCast(d->volumeActor->GetMapper()
                                                    ->GetInput());
    MeshColoration* coloration = new MeshColoration(
      d->videoConfig, d->videoPath, d->cameras);

    coloration->SetInput(volume);
    coloration->SetFrameSampling(d->UI.spinBoxFrameSampling->value());

    if(d->UI.radioButtonCurrentFrame->isChecked())
    {
      coloration->ProcessColoration(volume, d->currentFrame);
    }
    else
    {
      coloration->ProcessColoration(volume);
    }

    std::string name;
    int nbArray = volume->GetPointData()->GetNumberOfArrays();

    for (int i = 0; i < nbArray; ++i)
    {
      name = volume->GetPointData()->GetArrayName(i);
      d->UI.comboBoxColorDisplay->addItem(qtString(name));
    }

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
  d->UI.comboBoxColorDisplay->clear();
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
