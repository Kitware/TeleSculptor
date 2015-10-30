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

#include "FeatureOptions.h"

#include "ui_FeatureOptions.h"

#include "vtkMaptkFeatureTrackRepresentation.h"

#include <vtkActor.h>
#include <vtkProperty.h>

#include <qtUiState.h>
#include <qtUiStateItem.h>

//-----------------------------------------------------------------------------
class FeatureOptionsPrivate
{
public:
  void mapUiState(QString const& key, QSlider* slider);
  void mapUiState(QString const& key, QComboBox* comboBox);

  Ui::FeatureOptions UI;
  qtUiState uiState;

  vtkMaptkFeatureTrackRepresentation* representation;
};

QTE_IMPLEMENT_D_FUNC(FeatureOptions)

//-----------------------------------------------------------------------------
void FeatureOptionsPrivate::mapUiState(
  QString const& key, QSlider* slider)
{
  auto const item = new qtUiState::Item<int, QSlider>(
    slider, &QSlider::value, &QSlider::setValue);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
void FeatureOptionsPrivate::mapUiState(
  QString const& key, QComboBox* comboBox)
{
  auto const item = new qtUiState::Item<int, QComboBox>(
    comboBox, &QComboBox::currentIndex, &QComboBox::setCurrentIndex);
  this->uiState.map(key, item);
}

//-----------------------------------------------------------------------------
FeatureOptions::FeatureOptions(vtkMaptkFeatureTrackRepresentation* rep,
                               QString const& settingsGroup,
                               QWidget* parent, Qt::WindowFlags flags)
  : PointOptions(settingsGroup, parent, flags),
    d_ptr(new FeatureOptionsPrivate)
{
  QTE_D();

  // Set up UI
  auto const w = new QWidget(this);
  d->UI.setupUi(w);

  auto const layout = qobject_cast<QFormLayout*>(this->layout());
  layout->addRow(w);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.mapChecked("Trails", d->UI.showTrails);
  d->UI.trailColor->persist(d->uiState, "Trails/Color");
  d->mapUiState("Trails/Length", d->UI.trailLength);
  d->mapUiState("Trails/Style", d->UI.trailStyle);

  d->uiState.restore();

  // Set up initial representation state
  d->representation = rep;

  this->setTrailsVisible(d->UI.showTrails->isChecked());
  this->setTrailsLength(d->UI.trailLength->value());
  this->setTrailsStyle(d->UI.trailStyle->currentIndex());

  d->UI.trailColor->addActor(d->representation->GetTrailsActor());

  this->addActor(d->representation->GetActivePointsActor());

  // Connect signals/slots
  connect(d->UI.showTrails, SIGNAL(toggled(bool)),
          this, SLOT(setTrailsVisible(bool)));
  connect(d->UI.trailLength, SIGNAL(valueChanged(int)),
          this, SLOT(setTrailsLength(int)));
  connect(d->UI.trailStyle, SIGNAL(currentIndexChanged(int)),
          this, SLOT(setTrailsStyle(int)));
}

//-----------------------------------------------------------------------------
FeatureOptions::~FeatureOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setFeaturesVisible(bool state)
{
  QTE_D();

  d->representation->GetActivePointsActor()->SetVisibility(state);
  d->representation->GetTrailsActor()->SetVisibility(
    state && d->UI.showTrails->isChecked());

  emit this->modified();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setTrailsVisible(bool state)
{
  QTE_D();

  d->representation->GetTrailsActor()->SetVisibility(state);

  emit this->modified();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setTrailsLength(int length)
{
  QTE_D();

  d->representation->SetTrailLength(static_cast<unsigned>(length));

  emit this->modified();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setTrailsStyle(int style)
{
  QTE_D();

  d->representation->SetTrailStyle(
    static_cast<vtkMaptkFeatureTrackRepresentation::TrailStyleEnum>(style));

  emit this->modified();
}
