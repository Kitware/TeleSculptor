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
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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

#include "VisibleLandmarksOptions.h"

#include "ui_VisibleLandmarksOptions.h"

#include "DataColorOptions.h"
#include "DataFilterOptions.h"
#include "FieldInformation.h"

#include <vtkActor.h>
#include <vtkDataSet.h>
#include <vtkDataSetAttributes.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkThresholdPoints.h>

#include <qtEnumerate.h>
#include <qtScopedValueChange.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QMenu>
#include <QtGui/QWidgetAction>

namespace
{

//-----------------------------------------------------------------------------
enum ColorMode
{
  SolidColor,
  TrueColor,
  DataColor,
};

//-----------------------------------------------------------------------------
template <typename Container>
Container sorted(Container c)
{
  qSort(c);
  return c;
}

}

///////////////////////////////////////////////////////////////////////////////

//BEGIN VisibleLandmarksOptionsPrivate

QTE_IMPLEMENT_D_FUNC(VisibleLandmarksOptions)

//-----------------------------------------------------------------------------
class VisibleLandmarksOptionsPrivate
{
public:

  Ui::VisibleLandmarksOptions UI;
  qtUiState uiState;

  vtkActor* actor;

};

//END VisibleLandmarksOptionsPrivate

///////////////////////////////////////////////////////////////////////////////

//BEGIN VisibleLandmarksOptions

//-----------------------------------------------------------------------------
VisibleLandmarksOptions::VisibleLandmarksOptions(QString const& settingsGroup,
                           QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new VisibleLandmarksOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->UI.color->persist(d->uiState, "Color");

  auto const sizeItem = new qtUiState::Item<int, QSlider>(
    d->UI.size, &QSlider::value, &QSlider::setValue);
  d->uiState.map("Size", sizeItem);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.size, SIGNAL(valueChanged(int)), this, SLOT(setSize(int)));
  connect(d->UI.color, SIGNAL(colorChanged(QColor)), this, SIGNAL(visibleLandmarksChanged()));
}

//-----------------------------------------------------------------------------
VisibleLandmarksOptions::~VisibleLandmarksOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void VisibleLandmarksOptions::setDefaultColor(QColor const& color)
{
  QTE_D();

  d->UI.color->setColor(color);
  d->uiState.restore();
}

//-----------------------------------------------------------------------------
void VisibleLandmarksOptions::setVisibility(bool state)
{
  QTE_D();

  d->actor->SetVisibility(state);

  emit this->visibleLandmarksChanged();
}

//-----------------------------------------------------------------------------
void VisibleLandmarksOptions::addActor(vtkActor* actor)
{
  QTE_D();

  d->UI.color->addActor(actor);
  actor->GetProperty()->SetPointSize(d->UI.size->value());

  d->actor = actor;
}

//-----------------------------------------------------------------------------
void VisibleLandmarksOptions::setSize(int size)
{
  QTE_D();

  d->actor->GetProperty()->SetPointSize(size);

  emit this->visibleLandmarksChanged();
}

//END VisibleLandmarksOptions
