// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
