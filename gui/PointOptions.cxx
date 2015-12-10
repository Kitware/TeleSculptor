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

#include "PointOptions.h"

#include "ui_PointOptions.h"

#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkProperty.h>

#include <qtUiState.h>
#include <qtUiStateItem.h>

namespace
{

//-----------------------------------------------------------------------------
enum ColorMode
{
  SolidColor,
  TrueColor,
  DataColor,
};

}

//-----------------------------------------------------------------------------
class PointOptionsPrivate
{
public:
  Ui::PointOptions UI;
  qtUiState uiState;

  QList<vtkActor*> actors;
  QList<vtkMapper*> mappers;

  QButtonGroup colorMode;
};

QTE_IMPLEMENT_D_FUNC(PointOptions)

//-----------------------------------------------------------------------------
PointOptions::PointOptions(QString const& settingsGroup,
                           QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new PointOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->colorMode.addButton(d->UI.solidColor, SolidColor);
  d->colorMode.addButton(d->UI.trueColor, TrueColor);
  d->colorMode.addButton(d->UI.dataColor, DataColor);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->UI.color->persist(d->uiState, "Color");

  auto const sizeItem = new qtUiState::Item<int, QSlider>(
    d->UI.size, &QSlider::value, &QSlider::setValue);
  d->uiState.map("Size", sizeItem);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.color, SIGNAL(colorChanged(QColor)), this, SIGNAL(modified()));
  connect(d->UI.size, SIGNAL(valueChanged(int)), this, SLOT(setSize(int)));

  connect(&d->colorMode, SIGNAL(buttonClicked(int)),
          this, SLOT(setColorMode(int)));
}

//-----------------------------------------------------------------------------
PointOptions::~PointOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void PointOptions::setDefaultColor(QColor const& color)
{
  QTE_D();

  d->UI.color->setColor(color);
  d->uiState.restore();
}

//-----------------------------------------------------------------------------
void PointOptions::setTrueColorAvailable(bool available)
{
  QTE_D();

  d->UI.trueColor->setEnabled(available);
  if (!available && d->UI.trueColor->isChecked())
  {
    d->UI.solidColor->setChecked(true);
  }
}

//-----------------------------------------------------------------------------
void PointOptions::addActor(vtkActor* actor, vtkMapper* mapper)
{
  QTE_D();

  d->UI.color->addActor(actor);
  actor->GetProperty()->SetPointSize(d->UI.size->value());

  if (mapper)
  {
    mapper->SetScalarVisibility(!d->UI.solidColor->isChecked());
  }

  d->actors.append(actor);
  d->mappers.append(mapper);
}

//-----------------------------------------------------------------------------
void PointOptions::setSize(int size)
{
  QTE_D();

  foreach (auto const actor, d->actors)
  {
    actor->GetProperty()->SetPointSize(size);
  }

  emit this->modified();
}

//-----------------------------------------------------------------------------
void PointOptions::setColorMode(int mode)
{
  QTE_D();

  foreach (auto const mapper, d->mappers)
  {
    if (mapper)
    {
      mapper->SetScalarVisibility(mode != SolidColor);
    }
  }

  emit this->modified();
}
