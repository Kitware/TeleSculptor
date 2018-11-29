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

#include "GroundControlPointsView.h"

#include "ui_GroundControlPointsView.h"
#include "am_GroundControlPointsView.h"

#include <qtUtil.h>

#include <QMenu>
#include <QToolButton>

QTE_IMPLEMENT_D_FUNC(GroundControlPointsView)

//-----------------------------------------------------------------------------
class GroundControlPointsViewPrivate
{
public:
  Ui::GroundControlPointsView UI;
  Am::GroundControlPointsView AM;

  QMenu* popupMenu;
};

//-----------------------------------------------------------------------------
GroundControlPointsView::GroundControlPointsView(
  QWidget* parent, Qt::WindowFlags flags)
  : QWidget{parent, flags}, d_ptr{new GroundControlPointsViewPrivate}
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

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
