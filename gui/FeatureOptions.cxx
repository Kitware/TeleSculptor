// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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

  void toggleTrailOptions();

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
void FeatureOptionsPrivate::toggleTrailOptions()
{
  bool enable = this->UI.showTrailsWithDesc->isChecked() ||
                this->UI.showTrailsWithoutDesc->isChecked();
  this->UI.trailOptions->setEnabled(enable);
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

  d->uiState.mapChecked("TrailsWithDesc", d->UI.showTrailsWithDesc);
  d->uiState.mapChecked("TrailsWithoutDesc", d->UI.showTrailsWithoutDesc);
  d->UI.trailColorWithoutDesc->persist(d->uiState, "TrailsWithoutDesc/Color");
  d->UI.trailColorWithDesc->persist(d->uiState, "TrailsWithDesc/Color");
  d->mapUiState("Trails/Length", d->UI.trailLength);
  d->mapUiState("Trails/Style", d->UI.trailStyle);

  d->uiState.restore();

  // Set up initial representation state
  d->representation = rep;

  this->setTrailsWithDescVisible(d->UI.showTrailsWithDesc->isChecked());
  this->setTrailsWithoutDescVisible(d->UI.showTrailsWithoutDesc->isChecked());
  this->setTrailsLength(d->UI.trailLength->value());
  this->setTrailsStyle(d->UI.trailStyle->currentIndex());

  d->UI.trailColorWithDesc->addActor(
    d->representation->GetTrailsWithDescActor());
  d->UI.trailColorWithoutDesc->addActor(
    d->representation->GetTrailsWithoutDescActor());

  this->addActor(d->representation->GetActivePointsWithDescActor());
  this->addActor(d->representation->GetActivePointsWithoutDescActor());

  // Connect signals/slots
  connect(d->UI.showTrailsWithDesc, &QAbstractButton::toggled,
          this, &FeatureOptions::setTrailsWithDescVisible);
  connect(d->UI.showTrailsWithoutDesc, &QAbstractButton::toggled,
          this, &FeatureOptions::setTrailsWithoutDescVisible);
  connect(d->UI.trailLength, &QAbstractSlider::valueChanged,
          this, &FeatureOptions::setTrailsLength);
  connect(d->UI.trailStyle,
          QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &FeatureOptions::setTrailsStyle);
}

//-----------------------------------------------------------------------------
FeatureOptions::~FeatureOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setFeaturesWithDescVisible(bool state)
{
  QTE_D();

  d->representation->GetActivePointsWithDescActor()->SetVisibility(state);
  d->representation->GetTrailsWithDescActor()->SetVisibility(
    state && d->UI.showTrailsWithDesc->isChecked());

  emit this->modified();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setFeaturesWithoutDescVisible(bool state)
{
  QTE_D();

  d->representation->GetActivePointsWithoutDescActor()->SetVisibility(state);
  d->representation->GetTrailsWithoutDescActor()->SetVisibility(
    state && d->UI.showTrailsWithoutDesc->isChecked());

  emit this->modified();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setTrailsWithDescVisible(bool state)
{
  QTE_D();

  d->toggleTrailOptions();
  d->representation->GetTrailsWithDescActor()->SetVisibility(state);

  emit this->modified();
}

//-----------------------------------------------------------------------------
void FeatureOptions::setTrailsWithoutDescVisible(bool state)
{
  QTE_D();

  d->toggleTrailOptions();
  d->representation->GetTrailsWithoutDescActor()->SetVisibility(state);

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
