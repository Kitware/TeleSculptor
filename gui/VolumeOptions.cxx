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

#include "VolumeOptions.h"
#include "ui_VolumeOptions.h"
#include "ColorizeSurfaceOptions.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QMenu>
#include <QtGui/QWidgetAction>
#include <QToolButton>

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkPointData.h>

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
VolumeOptions::VolumeOptions(const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
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
  connect(d->UI.checkBoxColorizeSurface, SIGNAL(toggled(bool)),
          this, SLOT(showColorizeSurfaceMenu(bool)));

  connect(d->colorizeSurfaceOptions, SIGNAL(colorModeChanged(QString)),
          this, SLOT(updateColorizeSurfaceMenu(QString)));

  connect(d->UI.doubleSpinBoxSurfaceThreshold, SIGNAL(valueChanged(double)),
    parent, SLOT(computeContour(double)));

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

  vtkPointData* pointData = d->volumeActor->GetMapper()->GetInput()->GetPointData();
  int nbArray = pointData->GetNumberOfArrays();

  std::string name;

  for (int i = 0; i < nbArray; ++i) {

    name = pointData->GetArrayName(i);
    d->colorizeSurfaceOptions->addColorDisplay(name);
  }
}

//-----------------------------------------------------------------------------
void VolumeOptions::initFrameSampling(int nbFrames)
{
  QTE_D();

  d->colorizeSurfaceOptions->initFrameSampling(nbFrames);
}

//-----------------------------------------------------------------------------
void VolumeOptions::showColorizeSurfaceMenu(bool state)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setEnabled(state);
}

//-----------------------------------------------------------------------------
void VolumeOptions::updateColorizeSurfaceMenu(QString text)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setText(text);
}
