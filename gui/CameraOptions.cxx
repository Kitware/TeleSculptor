/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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

#include "CameraOptions.h"

#include "ui_CameraOptions.h"

#include "vtkMaptkCameraRepresentation.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

//-----------------------------------------------------------------------------
class CameraOptionsPrivate
{
public:
  void mapUiState(QString const& key, QSlider* slider);
  void mapUiState(QString const& key, qtDoubleSlider* slider);
  void mapUiState(QString const& key, qtColorButton* button);

  Ui::CameraOptions UI;
  qtUiState uiState;
};

QTE_IMPLEMENT_D_FUNC(CameraOptions)

//-----------------------------------------------------------------------------
void CameraOptionsPrivate::mapUiState(
  QString const& key, QSlider* slider)
{
  auto const item = new qtUiState::Item<int, QSlider>(
    slider, &QSlider::value, &QSlider::setValue);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
void CameraOptionsPrivate::mapUiState(
  QString const& key, qtDoubleSlider* slider)
{
  auto const item = new qtUiState::Item<double, qtDoubleSlider>(
    slider, &qtDoubleSlider::value, &qtDoubleSlider::setValue);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
void CameraOptionsPrivate::mapUiState(
  QString const& key, qtColorButton* button)
{
  auto const item = new qtUiState::Item<QColor, qtColorButton>(
    button, &qtColorButton::color, &qtColorButton::setColor);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
CameraOptions::CameraOptions(QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new CameraOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // TODO We may want to get a parent group from the user (of this class) that
  //      would identify which view in case of multiple views, so that we can
  //      have per-view persistence
  d->uiState.setCurrentGroup("Camera");

  d->mapUiState("Path/Color", d->UI.pathColor);
  d->mapUiState("Active/Color", d->UI.activeColor);
  d->mapUiState("Inactive/Color", d->UI.inactiveColor);

  d->mapUiState("Scale", d->UI.scale);
  d->mapUiState("Inactive/Scale", d->UI.inactiveScale);
  d->mapUiState("Inactive/PointSize", d->UI.inactivePointSize);

  d->uiState.mapChecked("Inactive", d->UI.showInactive);
  d->uiState.mapChecked("Inactive/Points", d->UI.inactiveAsPoints);
  d->uiState.mapChecked("Inactive/Frustums", d->UI.inactiveAsFrustums);
  d->uiState.mapChecked("Path", d->UI.inactiveAsFrustums);

  d->uiState.restore();
}

//-----------------------------------------------------------------------------
CameraOptions::~CameraOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void CameraOptions::setRepresentation(vtkMaptkCameraRepresentation* rep)
{
  // TODO
}
