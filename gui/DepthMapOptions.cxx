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

#include "DepthMapOptions.h"

#include "ui_DepthMapOptions.h"

#include "DepthMapFilterOptions.h"

#include <QMenu>
#include <QWidgetAction>

QTE_IMPLEMENT_D_FUNC(DepthMapOptions)

//-----------------------------------------------------------------------------
class DepthMapOptionsPrivate
{
public:
  void setPopup(QToolButton* button, QWidget* widget);

  Ui::DepthMapOptions UI;

  DepthMapFilterOptions* filterOptions;
};

//-----------------------------------------------------------------------------
void DepthMapOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//-----------------------------------------------------------------------------
DepthMapOptions::DepthMapOptions(QString const& settingsGroup,
                                 QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->filterOptions = new DepthMapFilterOptions(settingsGroup, this);
  d->setPopup(d->UI.filterMenu, d->filterOptions);
  d->UI.filterMenu->setEnabled(false);

  // Connect signals/slots
  connect(d->UI.points, SIGNAL(toggled(bool)),
          this, SIGNAL(displayModeChanged()));
  connect(d->UI.surfaces, SIGNAL(toggled(bool)),
          this, SIGNAL(displayModeChanged()));

  connect(d->UI.filter, SIGNAL(toggled(bool)),
          this, SIGNAL(thresholdsChanged(bool)));
  connect(d->filterOptions, SIGNAL(filtersChanged()),
          this, SLOT(thresholdsChanged()));
}

//-----------------------------------------------------------------------------
DepthMapOptions::~DepthMapOptions()
{
}

//-----------------------------------------------------------------------------
bool DepthMapOptions::isFilterPersistent() const
{
  QTE_D();
  return d->filterOptions->isFilterPersistent();
}

//-----------------------------------------------------------------------------
bool DepthMapOptions::isFilterEnabled() const
{
  QTE_D();
  return d->UI.filter->isChecked();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::bestCostValueMinimum() const
{
  QTE_D();
  return d->filterOptions->bestCostValueMinimum();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::bestCostValueMaximum() const
{
  QTE_D();
  return d->filterOptions->bestCostValueMaximum();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::uniquenessRatioMinimum() const
{
  QTE_D();
  return d->filterOptions->uniquenessRatioMinimum();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::uniquenessRatioMaximum() const
{
  QTE_D();
  return d->filterOptions->uniquenessRatioMaximum();
}

//-----------------------------------------------------------------------------
void DepthMapOptions::initializeFilters(double bcMin, double bcMax,
                                        double urMin, double urMax)
{
  QTE_D();
  d->filterOptions->initializeFilters(bcMin, bcMax, urMin, urMax);
}

//-----------------------------------------------------------------------------
DepthMapOptions::DisplayMode DepthMapOptions::displayMode() const
{
  QTE_D();
  return (d->UI.surfaces->isChecked() ? Surfaces : Points);
}

//-----------------------------------------------------------------------------
void DepthMapOptions::thresholdsChanged()
{
  QTE_D();
  emit thresholdsChanged(d->UI.filter->isChecked());
}
