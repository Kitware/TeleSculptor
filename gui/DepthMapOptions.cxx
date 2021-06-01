// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
  connect(d->UI.points, &QAbstractButton::toggled,
          this, &DepthMapOptions::displayModeChanged);
  connect(d->UI.surfaces, &QAbstractButton::toggled,
          this, &DepthMapOptions::displayModeChanged);

  connect(d->UI.filter, &QAbstractButton::toggled,
          this, QOverload<bool>::of(&DepthMapOptions::thresholdsChanged));
  connect(d->filterOptions, &DepthMapFilterOptions::filtersChanged,
          this, QOverload<>::of(&DepthMapOptions::thresholdsChanged));
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
double DepthMapOptions::weightMinimum() const
{
  QTE_D();
  return d->filterOptions->weightMinimum();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::weightMaximum() const
{
  QTE_D();
  return d->filterOptions->weightMaximum();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::uncertaintyMinimum() const
{
  QTE_D();
  return d->filterOptions->uncertaintyMinimum();
}

//-----------------------------------------------------------------------------
double DepthMapOptions::uncertaintyMaximum() const
{
  QTE_D();
  return d->filterOptions->uncertaintyMaximum();
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
