/*ckwg +29
 * Copyright 2016-2018 by Kitware, Inc.
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

#include "RulerOptions.h"

#include "ui_RulerOptions.h"

#include "RulerHelper.h"
#include "RulerWidget.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QDoubleSpinBox>
#include <QMenu>
#include <QToolButton>
#include <QWidgetAction>

///////////////////////////////////////////////////////////////////////////////

// BEGIN RulerOptionsPrivate

QTE_IMPLEMENT_D_FUNC(RulerOptions)

//-----------------------------------------------------------------------------
class RulerOptionsPrivate
{
public:
  void setPopup(QToolButton* button, QWidget* popup);

  RulerHelper* helper = nullptr;

  Ui::RulerOptions UI;
  qtUiState uiState;
};

//-----------------------------------------------------------------------------
void RulerOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

// END RulerOptionsPrivate

///////////////////////////////////////////////////////////////////////////////

// BEGIN RulerOptions

//-----------------------------------------------------------------------------
RulerOptions::RulerOptions(QString const& settingsGroup,
                           QWidget* parent,
                           Qt::WindowFlags flags)
  : QWidget(parent, flags)
  , d_ptr(new RulerOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->UI.rulerColor->persist(d->uiState, "RulerColor");

  auto const sizeItem = new qtUiState::Item<double, QDoubleSpinBox>(
    d->UI.unitDistance, &QDoubleSpinBox::value, &QDoubleSpinBox::setValue);
  d->uiState.map("UnitDistance", sizeItem);

  d->uiState.restore();

  d->UI.resetRuler->setDisabled(true);

  // Connect signals/slots
  QObject::connect(d->UI.resetRuler,
                   &QAbstractButton::clicked,
                   this,
                   &RulerOptions::resetRuler);
}

//-----------------------------------------------------------------------------
RulerOptions::~RulerOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void RulerOptions::setRulerHelper(RulerHelper* h)
{
  QTE_D();

  if (d->helper == h)
  {
    return;
  }

  if (!h && d->helper)
  {
    d->helper->disconnect();
  }
  d->helper = h;
  if (d->helper)
  {
    d->helper->setRulerTickDistance(d->UI.unitDistance->value());
    d->helper->setRulerColor(d->UI.rulerColor->color());

    connect(d->UI.resetRuler,
            &QAbstractButton::clicked,
            d->helper,
            &RulerHelper::resetRuler);
    connect(d->helper->worldWidget(),
            &RulerWidget::rulerPlaced,
            d->UI.resetRuler,
            &QAbstractButton::setEnabled);
    connect(d->UI.unitDistance,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            d->helper,
            &RulerHelper::setRulerTickDistance);
    connect(d->UI.rulerColor,
            &ActorColorButton::colorChanged,
            d->helper,
            &RulerHelper::setRulerColor);
  }
}

// END RulerOptions
