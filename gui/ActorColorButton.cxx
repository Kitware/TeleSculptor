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
