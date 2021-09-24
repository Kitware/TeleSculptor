// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "CameraOptions.h"

#include "ui_CameraOptions.h"

#include "vtkMaptkCameraRepresentation.h"

#include <vtkActor.h>
#include <vtkProperty.h>

#include <qtMath.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QButtonGroup>

namespace // anonymous
{

//-----------------------------------------------------------------------------
double scaleValue(qtDoubleSlider* slider)
{
  return qPow(10.0, slider->value());
}

}

//-----------------------------------------------------------------------------
class CameraOptionsPrivate
{
public:
  CameraOptionsPrivate() : visibility(true), baseCameraScale(10.0) {}

  double activeScale() const;
  double inactiveScale() const;

  void mapUiState(QString const& key, QSlider* slider);
  void mapUiState(QString const& key, qtDoubleSlider* slider);

  Ui::CameraOptions UI;
  qtUiState uiState;

  vtkMaptkCameraRepresentation* representation;

  bool visibility;
  double baseCameraScale;
};

QTE_IMPLEMENT_D_FUNC(CameraOptions)

//-----------------------------------------------------------------------------
double CameraOptionsPrivate::activeScale() const
{
  return this->baseCameraScale * this->UI.scale->value();
}

//-----------------------------------------------------------------------------
double CameraOptionsPrivate::inactiveScale() const
{
  return this->activeScale() * scaleValue(this->UI.inactiveScale);
}

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
CameraOptions::CameraOptions(vtkMaptkCameraRepresentation* rep,
                             QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new CameraOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Bundle inactive display modes into a group (mostly so we can have a single
  // signal when the mode changes, rather than toggled() for both the old and
  // new modes)
  auto const inactiveModeGroup = new QButtonGroup(this);
  inactiveModeGroup->addButton(d->UI.inactiveAsPoints);
  inactiveModeGroup->addButton(d->UI.inactiveAsFrustums);

  // Set default values for scale sliders
  d->UI.scale->setValue(0.25);
  d->UI.inactiveScale->setValue(-1.0);

  // Set up option persistence
  // TODO We may want to get a parent group from the user (of this class) that
  //      would identify which view in case of multiple views, so that we can
  //      have per-view persistence
  d->uiState.setCurrentGroup("Camera");

  d->UI.pathColor->persist(d->uiState, "Path/Color");
  d->UI.activeColor->persist(d->uiState, "Active/Color");
  d->UI.inactiveColor->persist(d->uiState, "Inactive/Color");

  d->mapUiState("Scale", d->UI.scale);
  d->mapUiState("Inactive/Scale", d->UI.inactiveScale);
  d->mapUiState("Inactive/PointSize", d->UI.inactivePointSize);

  d->uiState.mapChecked("Inactive", d->UI.showInactive);
  d->uiState.mapChecked("Inactive/Points", d->UI.inactiveAsPoints);
  d->uiState.mapChecked("Inactive/Frustums", d->UI.inactiveAsFrustums);
  d->uiState.mapChecked("Path", d->UI.showPath);

  d->uiState.restore();

  // Set up initial representation state
  d->representation = rep;

  d->UI.pathColor->addActor(rep->GetPathActor());
  d->UI.activeColor->addActor(rep->GetActiveActor());
  d->UI.inactiveColor->addActor(rep->GetNonActiveActor());

  updateScale();

  setPathVisible(d->UI.showPath->isChecked());
  setInactiveVisible(d->UI.showInactive->isChecked());

  // Connect signals/slots
  connect(d->UI.pathColor, &qtColorButton::colorChanged,
          this, &CameraOptions::modified);
  connect(d->UI.activeColor, &qtColorButton::colorChanged,
          this, &CameraOptions::modified);
  connect(d->UI.inactiveColor, &qtColorButton::colorChanged,
          this, &CameraOptions::modified);

  connect(d->UI.scale, &qtDoubleSlider::valueChanged,
          this, &CameraOptions::updateScale);

  connect(inactiveModeGroup,
          QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
          this, &CameraOptions::updateInactiveDisplayOptions);
  connect(d->UI.inactivePointSize, &QSlider::valueChanged,
          this, &CameraOptions::updateInactiveDisplayOptions);
  connect(d->UI.inactiveScale, &qtDoubleSlider::valueChanged,
          this, &CameraOptions::updateInactiveDisplayOptions);

  connect(d->UI.showPath, &QAbstractButton::toggled,
          this, &CameraOptions::setPathVisible);
  connect(d->UI.showInactive, &QAbstractButton::toggled,
          this, &CameraOptions::setInactiveVisible);
}

//-----------------------------------------------------------------------------
CameraOptions::~CameraOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void CameraOptions::setBaseCameraScale(double s)
{
  QTE_D();

  if (s != d->baseCameraScale)
  {
    d->baseCameraScale = s;
    this->updateScale();
  }
}

//-----------------------------------------------------------------------------
void CameraOptions::setCamerasVisible(bool state)
{
  QTE_D();

  d->visibility = state;

  d->representation->GetActiveActor()->SetVisibility(state);
  d->representation->GetNonActiveActor()->SetVisibility(
    state && d->UI.showInactive->isChecked());
  d->representation->GetPathActor()->SetVisibility(
    state && d->UI.showPath->isChecked());

  emit this->modified();
}

//-----------------------------------------------------------------------------
void CameraOptions::setPathVisible(bool state)
{
  QTE_D();

  d->representation->GetPathActor()->SetVisibility(d->visibility && state);

  emit this->modified();
}

//-----------------------------------------------------------------------------
void CameraOptions::setInactiveVisible(bool state)
{
  QTE_D();

  d->representation->GetNonActiveActor()->SetVisibility(d->visibility && state);

  emit this->modified();
}

//-----------------------------------------------------------------------------
void CameraOptions::updateScale()
{
  QTE_D();

  d->representation->SetActiveCameraRepLength(d->activeScale());

  if (d->UI.inactiveAsFrustums->isChecked())
  {
    // Also update inactive scale; this will update the representation (which
    // we don't want to do twice, because it's expensive) and emit
    // this->modified()
    this->updateInactiveDisplayOptions();
  }
  else
  {
    d->representation->Update();
    emit this->modified();
  }
}

//-----------------------------------------------------------------------------
void CameraOptions::updateInactiveDisplayOptions()
{
  QTE_D();

  if (d->UI.inactiveAsFrustums->isChecked())
  {
    // TODO set display mode to frustums
    d->representation->SetNonActiveCameraRepLength(d->inactiveScale());
    d->representation->Update();
  }
  else
  {
    // TODO
  }

  emit this->modified();
}
