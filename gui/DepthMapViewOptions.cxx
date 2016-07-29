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

#include "DepthMapViewOptions.h"

#include "ui_DepthMapViewOptions.h"

#include "DataArrays.h"
#include "DataColorOptions.h"

#include <vtkActor.h>
#include <vtkDataArray.h>
#include <vtkMapper.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <QtGui/QMenu>
#include <QtGui/QWidgetAction>

QTE_IMPLEMENT_D_FUNC(DepthMapViewOptions)

//-----------------------------------------------------------------------------
class DepthMapViewOptionsPrivate
{
public:
  struct ModeInformation
  {
    char const* arrayName;
    DataColorOptions* options;
  };

public:
  DepthMapViewOptionsPrivate() : actor(0), rangeInitialized(false) {}

  void setPopup(QToolButton* button, QWidget* widget);

  void addMode(QAbstractButton* button, char const* arrayName,
               DataColorOptions* options);

  Ui::DepthMapViewOptions UI;

  DataColorOptions* depthOptions;
  DataColorOptions* bestCostValueOptions;
  DataColorOptions* uniquenessRatioOptions;

  QButtonGroup* modeButtons;
  QList<ModeInformation> modes;

  vtkActor* actor;

  bool rangeInitialized;
};

//-----------------------------------------------------------------------------
void DepthMapViewOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//-----------------------------------------------------------------------------
void DepthMapViewOptionsPrivate::addMode(
  QAbstractButton* button, char const* arrayName, DataColorOptions* options)
{
  auto index = this->modes.count();
  this->modeButtons->addButton(button, index);
  this->modes.append(ModeInformation{arrayName, options});
}

//-----------------------------------------------------------------------------
DepthMapViewOptions::DepthMapViewOptions(
  QString const& settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new DepthMapViewOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->depthOptions =
    new DataColorOptions(settingsGroup + "/Depth", this);
  d->setPopup(d->UI.depthOptions, d->depthOptions);
  setDepthIcon(d->depthOptions->icon());
  connect(d->depthOptions, SIGNAL(iconChanged(QIcon)),
          this, SLOT(setDepthIcon(QIcon)));

  d->bestCostValueOptions =
    new DataColorOptions(settingsGroup + "/BestCostValue", this);
  d->setPopup(d->UI.bestCostValueOptions, d->bestCostValueOptions);
  setBestCostValueIcon(d->bestCostValueOptions->icon());
  connect(d->bestCostValueOptions, SIGNAL(iconChanged(QIcon)),
          this, SLOT(setBestCostValueIcon(QIcon)));

  d->uniquenessRatioOptions =
    new DataColorOptions(settingsGroup + "/UniquenessRatio", this);
  d->setPopup(d->UI.uniquenessRatioOptions, d->uniquenessRatioOptions);
  setUniquenessRatioIcon(d->uniquenessRatioOptions->icon());
  connect(d->uniquenessRatioOptions, SIGNAL(iconChanged(QIcon)),
          this, SLOT(setUniquenessRatioIcon(QIcon)));

  d->modeButtons = new QButtonGroup(this);
  d->addMode(d->UI.color, DepthMapArrays::TrueColor, 0);
  d->addMode(d->UI.depth, DepthMapArrays::Depth,
             d->depthOptions);
  d->addMode(d->UI.bestCostValue, DepthMapArrays::BestCostValues,
             d->bestCostValueOptions);
  d->addMode(d->UI.uniquenessRatio, DepthMapArrays::UniquenessRatios,
             d->uniquenessRatioOptions);

  connect(d->depthOptions, SIGNAL(modified()),
          this, SLOT(updateActor()));
  connect(d->bestCostValueOptions, SIGNAL(modified()),
          this, SLOT(updateActor()));
  connect(d->uniquenessRatioOptions, SIGNAL(modified()),
          this, SLOT(updateActor()));

  connect(d->modeButtons, SIGNAL(buttonClicked(int)),
          this, SLOT(updateActor()));
}

//-----------------------------------------------------------------------------
DepthMapViewOptions::~DepthMapViewOptions()
{
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::setActor(vtkActor* actor)
{
  QTE_D();
  d->actor = actor;
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::updateRanges(vtkPointData* pointData)
{
  QTE_D();

  foreach (auto& mi, d->modes)
  {
    if (mi.options)
    {
      auto const dataArray = pointData->GetArray(mi.arrayName);
      if (dataArray)
      {
        double range[2];
        dataArray->GetRange(range);

        auto cumulativeLower = range[0];
        auto cumulativeUpper = range[1];
        if (d->rangeInitialized)
        {
          cumulativeLower = qMin(cumulativeLower, mi.options->minimum());
          cumulativeUpper = qMax(cumulativeUpper, mi.options->maximum());
        }

        mi.options->setAvailableRange(cumulativeLower, cumulativeUpper,
                                      range[0], range[1]);
      }
    }
  }

  d->rangeInitialized = true;
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::updateActor()
{
  QTE_D();

  auto const mode = d->modeButtons->checkedId();
  Q_ASSERT(mode >= 0 && mode < d->modes.count());

  auto const& mi = d->modes[mode];

  // Set active data on mapper
  auto const mapper = d->actor->GetMapper();
  auto const pointData = mapper->GetInput()->GetPointData();

  pointData->SetActiveScalars(mi.arrayName);
  mapper->SetScalarModeToUsePointData();

  if (mi.options)
  {
    mapper->SetColorModeToMapScalars();
    mapper->SetLookupTable(mi.options->scalarsToColors());
    mapper->UseLookupTableScalarRangeOn();
  }
  else
  {
    mapper->SetColorModeToDirectScalars();
    mapper->CreateDefaultLookupTable();
  }

  mapper->Update();
  emit this->modified();
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::setDepthIcon(QIcon const& icon)
{
  QTE_D();
  d->UI.depthOptions->setIcon(icon);
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::setBestCostValueIcon(QIcon const& icon)
{
  QTE_D();
  d->UI.bestCostValueOptions->setIcon(icon);
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::setUniquenessRatioIcon(QIcon const& icon)
{
  QTE_D();
  d->UI.uniquenessRatioOptions->setIcon(icon);
}
