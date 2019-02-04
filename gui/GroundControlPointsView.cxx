/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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

#include "GroundControlPointsView.h"

#include "ui_GroundControlPointsView.h"
#include "am_GroundControlPointsView.h"

#include "GroundControlPointsHelper.h"
#include "GroundControlPointsModel.h"

#include <qtUtil.h>

#include <QEvent>
#include <QFile>
#include <QMenu>
#include <QPainter>
#include <QScreen>
#include <QSvgRenderer>
#include <QToolButton>
#include <QWindow>

namespace
{

//-----------------------------------------------------------------------------
QPixmap colorize(QByteArray svg, int physicalSize, int logicalSize,
                 double devicePixelRatio, QColor color)
{
  svg.replace("#ffffff", color.name().toLatin1());

  QPixmap p{physicalSize, physicalSize};
  p.setDevicePixelRatio(devicePixelRatio);
  p.fill(Qt::transparent);

  QSvgRenderer renderer{svg};
  QPainter painter{&p};
  renderer.render(&painter, QRect{0, 0, logicalSize, logicalSize});

  return p;
}

}

//-----------------------------------------------------------------------------
class GroundControlPointsViewPrivate
{
public:
  void updateRegisteredIcon(QWidget* widget);

  Ui::GroundControlPointsView UI;
  Am::GroundControlPointsView AM;

  QMenu* popupMenu;

  GroundControlPointsModel model;
  GroundControlPointsHelper* helper = nullptr;
};

QTE_IMPLEMENT_D_FUNC(GroundControlPointsView)

//-----------------------------------------------------------------------------
void GroundControlPointsViewPrivate::updateRegisteredIcon(QWidget* widget)
{
  QIcon icon;

  auto const& palette = widget->palette();
  auto const normalColor =
    palette.color(QPalette::Active, QPalette::Text);
  auto const selectedColor =
    palette.color(QPalette::Active, QPalette::HighlightedText);
  auto const disabledColor =
    palette.color(QPalette::Disabled, QPalette::Text);

  QFile f{QStringLiteral(":/icons/scalable/registered")};
  f.open(QIODevice::ReadOnly);
  auto const svg = f.readAll();

  auto const dpr = widget->devicePixelRatioF();
  for (auto const size : {16, 20, 22, 24, 32})
  {
    auto const dsize = static_cast<int>(size * dpr);

    icon.addPixmap(colorize(svg, dsize, size, dpr, normalColor),
                   QIcon::Normal);
    icon.addPixmap(colorize(svg, dsize, size, dpr, selectedColor),
                   QIcon::Selected);
    icon.addPixmap(colorize(svg, dsize, size, dpr, disabledColor),
                   QIcon::Disabled);
  }

  this->model.setRegisteredIcon(icon);
}

//-----------------------------------------------------------------------------
GroundControlPointsView::GroundControlPointsView(
  QWidget* parent, Qt::WindowFlags flags)
  : QWidget{parent, flags}, d_ptr{new GroundControlPointsViewPrivate}
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  d->UI.pointsList->setModel(&d->model);

  d->updateRegisteredIcon(this);

  connect(this->window()->windowHandle(), &QWindow::screenChanged,
          this, [d, this]{ d->updateRegisteredIcon(this); });

  auto const clText = QStringLiteral("Copy Location");

  auto* const clMenu = new QMenu{clText, this};
  clMenu->addAction(d->UI.actionCopyLocationLatLon);
  clMenu->addAction(d->UI.actionCopyLocationLatLonElev);
  clMenu->addAction(d->UI.actionCopyLocationLonLat);
  clMenu->addAction(d->UI.actionCopyLocationLonLatElev);

  auto* const clButton = new QToolButton{d->UI.toolBar};
  clButton->setText(clText);
  clButton->setToolTip(clText);
  clButton->setIcon(
    qtUtil::standardActionIcon(QStringLiteral("copy-location")));
  clButton->setMenu(clMenu);
  clButton->setPopupMode(QToolButton::InstantPopup);

  auto* const spacer = new QWidget{d->UI.toolBar};
  spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  d->UI.toolBar->insertWidget(d->UI.actionRevert, clButton);
  d->UI.toolBar->insertWidget(d->UI.actionRevert, spacer);

  d->popupMenu = new QMenu{this};
  d->popupMenu->addMenu(clMenu);
  d->popupMenu->addAction(d->UI.actionRevert);
  d->popupMenu->addAction(d->UI.actionDelete);
}

//-----------------------------------------------------------------------------
GroundControlPointsView::~GroundControlPointsView()
{
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::setHelper(GroundControlPointsHelper* helper)
{
  QTE_D();

  if (d->helper)
  {
    disconnect(d->helper, nullptr, this, nullptr);
    disconnect(d->helper, nullptr, &d->model, nullptr);
  }

  d->helper = helper;

  connect(helper, &GroundControlPointsHelper::pointAdded,
          &d->model, &GroundControlPointsModel::addPoint);
  connect(helper, &GroundControlPointsHelper::pointRemoved,
          &d->model, &GroundControlPointsModel::removePoint);
  connect(helper, &GroundControlPointsHelper::pointsReloaded,
          &d->model, &GroundControlPointsModel::resetPoints);

  d->model.setPointData(helper->groundControlPoints());
}

//-----------------------------------------------------------------------------
void GroundControlPointsView::changeEvent(QEvent* e)
{
  if (e && e->type() == QEvent::PaletteChange)
  {
    QTE_D();
    d->updateRegisteredIcon(this);
  }

  QWidget::changeEvent(e);
}
