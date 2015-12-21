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

#include "FieldInformation.h"

#include <vtkActor.h>
#include <vtkDataSet.h>
#include <vtkDataSetAttributes.h>
#include <vtkMapper.h>
#include <vtkProperty.h>

#include <qtUiState.h>
#include <qtUiStateItem.h>

QTE_IMPLEMENT_D_FUNC(PointOptions)

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
  FieldInformation activeField() const;

  void setDisplayMode(vtkMapper*, int mode);

  Ui::PointOptions UI;
  qtUiState uiState;

  QList<vtkActor*> actors;
  QList<vtkMapper*> mappers;

  QHash<QString, FieldInformation> fields;

  QButtonGroup colorMode;
};

//-----------------------------------------------------------------------------
FieldInformation PointOptionsPrivate::activeField() const
{
  return this->fields[this->UI.dataField->currentText()];
}

//-----------------------------------------------------------------------------
void PointOptionsPrivate::setDisplayMode(vtkMapper* mapper, int mode)
{
  if (!mapper)
  {
    return;
  }

  auto const attr = mapper->GetInput()->GetAttributes(vtkDataObject::POINT);

  switch (mode)
  {
    case TrueColor:
      mapper->SetScalarVisibility(true);
      attr->SetActiveScalars("truecolor");
      break;

    case DataColor:
    {
      auto const& fi = this->activeField();
      mapper->SetScalarVisibility(true);
      mapper->SetScalarRange(fi.range[0], fi.range[1]);
      attr->SetActiveScalars(fi.name.constData());
      // TODO set colors
      break;
    }

    default:
      mapper->SetScalarVisibility(false);
      break;
  }
}

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

  connect(d->UI.dataField, SIGNAL(currentIndexChanged(int)),
          this, SLOT(setColorField(int)));

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
void PointOptions::setDataFields(
  QHash<QString, FieldInformation> const& fields)
{
  QTE_D();

  d->fields = fields;
  auto const haveFields = !fields.isEmpty();

  d->UI.dataColor->setEnabled(haveFields);
  if (!haveFields && d->UI.dataColor->isChecked())
  {
    d->UI.solidColor->setChecked(true);
  }

  d->UI.dataField->clear();
  foreach (auto const& fieldDisplayText, fields.keys())
  {
    d->UI.dataField->addItem(fieldDisplayText);
  }
}

//-----------------------------------------------------------------------------
void PointOptions::addActor(vtkActor* actor)
{
  QTE_D();

  d->UI.color->addActor(actor);
  actor->GetProperty()->SetPointSize(d->UI.size->value());

  d->actors.append(actor);
}

//-----------------------------------------------------------------------------
void PointOptions::addMapper(vtkMapper* mapper)
{
  QTE_D();

  d->setDisplayMode(mapper, d->colorMode.checkedId());

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
    d->setDisplayMode(mapper, mode);
  }

  emit this->modified();
}

//-----------------------------------------------------------------------------
void PointOptions::setColorField(int index)
{
  QTE_D();

  auto const mode = d->colorMode.checkedId();
  foreach (auto const mapper, d->mappers)
  {
    d->setDisplayMode(mapper, mode);
  }

  emit this->modified();
}
