// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "ActorColorButton.h"

#include <vtkActor.h>
#include <vtkProperty.h>

#include <qtUiState.h>
#include <qtUiStateItem.h>

namespace // anonymous
{

//-----------------------------------------------------------------------------
void setColor(vtkActor* actor, QColor const& color)
{
  auto const prop = actor->GetProperty();
  prop->SetColor(color.redF(), color.greenF(), color.blueF());
  prop->SetOpacity(color.alphaF());
}

}

//-----------------------------------------------------------------------------
class ActorColorButtonPrivate
{
public:
  QList<vtkActor*> actors;
};

QTE_IMPLEMENT_D_FUNC(ActorColorButton)

//-----------------------------------------------------------------------------
ActorColorButton::ActorColorButton(QWidget* parent)
  : qtColorButton(parent), d_ptr(new ActorColorButtonPrivate)
{
}

//-----------------------------------------------------------------------------
ActorColorButton::~ActorColorButton()
{
}

//-----------------------------------------------------------------------------
void ActorColorButton::addActor(vtkActor* actor)
{
  QTE_D();

  ::setColor(actor, this->color());

  d->actors.append(actor);
}

//-----------------------------------------------------------------------------
void ActorColorButton::persist(qtUiState& uiState, QString const& settingsKey)
{
  auto const item = new qtUiState::Item<QColor, qtColorButton>(
    this, &qtColorButton::color, &qtColorButton::setColor);
  uiState.map(settingsKey, item);
}

//-----------------------------------------------------------------------------
void ActorColorButton::setColor(QColor newColor)
{
  if (newColor != this->color())
  {
    QTE_D();

    foreach (auto const actor, d->actors)
    {
      ::setColor(actor, newColor);
    }

    qtColorButton::setColor(newColor);
  }
}
