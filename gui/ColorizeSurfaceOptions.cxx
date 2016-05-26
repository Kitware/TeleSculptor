/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include <qdebug.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <vtkActor.h>

#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

//-----------------------------------------------------------------------------
class ColorizeSurfaceOptionsPrivate
{
public:
  ColorizeSurfaceOptionsPrivate() {}

  Ui::ColorizeSurfaceOptions UI;
  qtUiState uiState;

  vtkActor* volumeActor;

  QString krtdFile;
  QString vtiFile;

  int currentFrameID;
};

QTE_IMPLEMENT_D_FUNC(ColorizeSurfaceOptions)

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::ColorizeSurfaceOptions(const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new ColorizeSurfaceOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);


  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.radioButtonCurrentFrame, SIGNAL(clicked()),
    this, SLOT(currentFrameSelected()));

  connect(d->UI.radioButtonAllFrames, SIGNAL(clicked()),
    this, SLOT(allFrameSelected()));


  connect(d->UI.buttonCompute, SIGNAL(clicked()),
    this, SLOT(colorize()));

  d->krtdFile = QString();
  d->vtiFile = QString();

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
  d->UI.spinBoxFrameSampling->setValue(nbFrames-1/20);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setActor(vtkActor* actor)
{
  QTE_D();

  d->volumeActor = actor;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setKrtdFile(QString file)
{
  QTE_D();

  d->krtdFile = file;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::setVtiFile(QString file)
{
  QTE_D();

  d->vtiFile = file;
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::colorize()
{
  QTE_D();
  if (d->UI.comboBoxColorDisplay->isEnabled() && !d->vtiFile.isEmpty() && !d->krtdFile.isEmpty())
  {
    vtkPolyData* volume = vtkPolyData::SafeDownCast(d->volumeActor->GetMapper()->GetInput());
    MeshColoration* coloration = new MeshColoration(volume, d->vtiFile.toStdString(), d->krtdFile.toStdString());
    coloration->SetInput(volume);
    coloration->SetFrameSampling(d->UI.spinBoxFrameSampling->value());
    coloration->ProcessColoration();

    std::string name;
    int nbArray = volume->GetPointData()->GetNumberOfArrays();
    for (int i = 0; i < nbArray; ++i)
    {
      name = volume->GetPointData()->GetArrayName(i);
      d->UI.comboBoxColorDisplay->addItem(QString(name.c_str()));
    }

    volume->GetPointData()->SetActiveScalars("MeanColoration");


  }

  emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
  emit(meshColorizedInColorizeSurfaceOption());
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::enableAllFramesParameters(bool state)
{
  QTE_D();

  d->UI.buttonCompute->setEnabled(state);
  d->UI.comboBoxColorDisplay->setEnabled(state);
  d->UI.spinBoxFrameSampling->setEnabled(state);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::allFrameSelected()
{
  this->enableAllFramesParameters(true);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::currentFrameSelected()
{
  this->enableAllFramesParameters(false);
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::updateCurrentFrameNumber(int idFrame)
{
  QTE_D();

  d->currentFrameID = idFrame;
  qDebug() << idFrame;
}
