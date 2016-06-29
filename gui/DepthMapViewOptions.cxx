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

#include "DepthMapViewOptions.h"
#include "ui_DepthMapViewOptions.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QRadioButton>
#include <QToolButton>
#include <QWidgetAction>
#include <QMenu>
#include "DataColorOptions.h"

#include <vtkImageActor.h>
#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkDataArray.h>


//-----------------------------------------------------------------------------
class DepthMapViewOptionsPrivate
{
public:
  Ui::DepthMapViewOptions UI;
  qtUiState uiState;

  vtkImageActor* imageActor;

  void setPopup(QToolButton* button, QWidget* widget);

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
QTE_IMPLEMENT_D_FUNC(DepthMapViewOptions)

DepthMapViewOptions::DepthMapViewOptions(QString const& settingsGroup,
                                 QWidget* parent, Qt::WindowFlags flags) :
  QWidget(parent, flags), d_ptr(new DepthMapViewOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  layout = new QFormLayout();
  d->UI.groupBox->setLayout(layout);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  bGroup = new QButtonGroup(d->UI.groupBox);
  // Connect signals/slots

}

DepthMapViewOptions::~DepthMapViewOptions()
{
  QTE_D();
  d->uiState.save();
}

void DepthMapViewOptions::addDepthMapMode(std::string name, bool needGradient)
{
  QTE_D();

  QRadioButton *scalar = new QRadioButton(QString::fromStdString(name));

  if (needGradient){
    QToolButton *gradient = new QToolButton();
    DataColorOptions *dataColorOptions = new DataColorOptions("DepthMapViewOptions/"+QString::fromStdString(name),
                                                              this);
    d->setPopup(gradient, dataColorOptions);
    layout->addRow(scalar,gradient);
    gradient->setIcon(dataColorOptions->icon());
  }
  else {
    layout->addRow(scalar);
  }



//  d->UI.groupBox->layout()->addWidget(button);

  scalar->setVisible(true);
  scalar->setEnabled(true);
  scalar->setCheckable(true);

  bGroup->addButton(scalar,bGroup->buttons().size());

  connect(scalar, SIGNAL(toggled(bool)),
          this, SLOT(switchDisplayMode(bool)));

//  connect(dataColorOptions, SIGNAL(modified()), this, SIGNAL(modified()));

  if (!bGroup->button(0)->isChecked())
  {
      bGroup->button(0)->setChecked(true);
  }

}

void DepthMapViewOptions::switchDisplayMode(bool checked)
{
  QTE_D();

  //This way it's only triggered on the checked event and not on the unchecked too
  if (checked)
  {
    int buttonId = bGroup->checkedId();
    std::cout << "buttonId = " << buttonId <<std::endl;
    //Displaying the scalar array associated with the checked radio button
    d->imageActor->GetInput()->GetPointData()
        ->SetScalars(d->imageActor->GetInput()
                     ->GetPointData()->GetArray(buttonId));

    emit this->modified();
  }
}

void DepthMapViewOptions::addActor(vtkImageActor *actor)
{
  QTE_D();

  bool needGradient;
  d->imageActor = actor;

  for (int i = 0; i < d->imageActor->GetInput()->GetPointData()->GetNumberOfArrays(); ++i)
  {
    if(d->imageActor->GetInput()->GetPointData()->GetArray(i)->GetNumberOfComponents() == 3)
    {
      needGradient = false;
    }
    else
    {
      needGradient = true;
    }

    addDepthMapMode(d->imageActor->GetInput()->GetPointData()->GetArrayName(i),needGradient);
  }
}

void DepthMapViewOptions::cleanModes()
{
  for (int i = 0; i < bGroup->buttons().size(); ++i) {
    bGroup->removeButton(bGroup->button(i));
  }
}

