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

#include <vtkPointData.h>
#include <vtkImageData.h>
#include <vtkDataArray.h>

#include <vtkScalarsToColors.h>
#include <vtkImageMapper3D.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToColors.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>


///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapViewOptionsPrivate declaration

//-----------------------------------------------------------------------------
class DepthMapViewOptionsPrivate
{
public:
  Ui::DepthMapViewOptions UI;
  qtUiState uiState;

  vtkActor* polyDataActor;
  vtkSmartPointer<vtkPolyData> originalImage;

  std::map<std::string, DataColorOptions*> dcOptions;
  std::map<std::string, QToolButton*> gradients;

  int lastButtonId;

  void setPopup(QToolButton* button, QWidget* widget);

};

//END DepthMapViewOptionsPrivate declaration

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapViewOptionsPrivate implementation

//-----------------------------------------------------------------------------
void DepthMapViewOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);


  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//END DepthMapViewOptionsPrivate declaration

///////////////////////////////////////////////////////////////////////////////

//BEGIN DepthMapViewOptions

QTE_IMPLEMENT_D_FUNC(DepthMapViewOptions)

//-----------------------------------------------------------------------------
DepthMapViewOptions::DepthMapViewOptions(QString const& settingsGroup,
                                 QWidget* parent, Qt::WindowFlags flags) :
  QWidget(parent, flags), d_ptr(new DepthMapViewOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  layout = new QVBoxLayout(d->UI.groupBox);
  d->UI.groupBox->setLayout(layout);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  bGroup = new QButtonGroup(d->UI.groupBox);
  d->uiState.restore();

  d->originalImage = vtkSmartPointer<vtkPolyData>::New();

  d->lastButtonId = 0;
}

//-----------------------------------------------------------------------------
DepthMapViewOptions::~DepthMapViewOptions()
{
  QTE_D();

  d->uiState.save();
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::addDepthMapMode(std::string name, bool needGradient,
                                          double lower, double upper)
{
  QTE_D();

  QRadioButton *scalar = new QRadioButton(QString::fromStdString(name));

  int index = bGroup->buttons().size();

  bGroup->addButton(scalar,index);

  if (needGradient)
  {
    QToolButton *gradient = new QToolButton(d->UI.groupBox);

    gradient->setPopupMode(gradient->InstantPopup);
    gradient->setEnabled(false);

    DataColorOptions *dataColorOptions =
        new DataColorOptions("DepthMapViewOptions/"+QString::fromStdString(name),this);

    gradient->setIcon(dataColorOptions->icon());

    d->setPopup(gradient, dataColorOptions);

    dataColorOptions->setEnabled(true);
    dataColorOptions->setAvailableRange(lower, upper);

    connect(dataColorOptions, SIGNAL(modified()),
            this, SIGNAL(modified()));

    connect(dataColorOptions, SIGNAL(modified()),
            this, SLOT(updateGradient()));

    QHBoxLayout *hLayout = new QHBoxLayout();

    hLayout->addWidget(scalar);
    hLayout->addWidget(gradient);

    layout->addLayout(hLayout);

    d->dcOptions.insert(std::pair<std::string, DataColorOptions*>(name,
                                                                  dataColorOptions));

    d->gradients.insert(std::pair<std::string, QToolButton*>(name, gradient));
  }
  else
  {
    layout->addWidget(scalar);
  }

  scalar->setVisible(true);
  scalar->setEnabled(true);
  scalar->setCheckable(true);

  connect(scalar, SIGNAL(toggled(bool)),
          this, SLOT(switchDisplayMode(bool)));
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::switchDisplayMode(bool checked)
{
  QTE_D();

  //This way it's only triggered on the checked event and not on the unchecked too

  if (checked)
  {
    std::string buttonId = bGroup->checkedButton()->text().toStdString();

    //Displaying the scalar array associated with the checked radio button

    vtkMapper* mapper = d->polyDataActor->GetMapper();
    vtkDataArray* activeArray =
        mapper->GetInput()->GetPointData()->GetArray(buttonId.c_str());

    mapper->GetInput()->GetPointData()->SetActiveScalars(buttonId.c_str());

    mapper->SetScalarModeToUsePointData();
    int numberOfComponents = activeArray->GetNumberOfComponents();

    if (numberOfComponents < 3)
    {
      //Only enable the gradient associated with the checked display mode

      std::map<std::string, QToolButton*>::iterator it;

      for (it = d->gradients.begin(); it != d->gradients.end(); ++it) {
        if(it->first != buttonId)
        {
          it->second->setEnabled(false);
        }
        else
        {
          it->second->setEnabled(true);
        }
      }

      DataColorOptions *dc = d->dcOptions.at(buttonId);

      double range[2];

      activeArray->GetRange(range);

      mapper->SetColorModeToMapScalars();
      mapper->SetLookupTable(dc->scalarsToColors());
      mapper->GetLookupTable()->SetRange(range);
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
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::updateGradient()
{
  QTE_D();

  std::string buttonId = bGroup->checkedButton()->text().toStdString();

  QToolButton *gradient = d->gradients.at(buttonId);

  DataColorOptions *dc = d->dcOptions.at(buttonId);

  gradient->setIcon(dc->icon());
  gradient->repaint();

  this->update();
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::addActor(vtkActor *polyDataActor)
{
  QTE_D();

  d->polyDataActor = polyDataActor;

  cleanModes();

  bool needGradient;

  d->originalImage->DeepCopy(d->polyDataActor->GetMapper()->GetInput());

  vtkPointData* pointData =
      d->polyDataActor->GetMapper()->GetInput()->GetPointData();

  for (int i = 0; i <  pointData->GetNumberOfArrays(); ++i)
  {
    double range[2];

    if(pointData->GetArray(i)->GetNumberOfComponents() == 3)
    {
      needGradient = false;
    }
    else
    {
      needGradient = true;
      pointData->GetArray(i)->GetRange(range);
    }

    addDepthMapMode(pointData->GetArrayName(i),needGradient, range[0], range[1]);
  }

  bGroup->buttons()[d->lastButtonId]->setChecked(true);
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::clearLayout(QLayout* layout)
{
  QLayoutItem* child;
  while(layout->count()!=0)
  {
    child = layout->takeAt(0);

    if(child->layout() != 0)
    {
      clearLayout(child->layout());
    }
    else if(child->widget() != 0)
    {
      delete child->widget();
    }

    delete child;
  }
}

//-----------------------------------------------------------------------------
void DepthMapViewOptions::cleanModes()
{
  QTE_D();

  if(bGroup)
  {
    if (bGroup->checkedId() >= 0)
    {
      d->lastButtonId = bGroup->checkedId();
    }

    for (int i = 0; i < bGroup->buttons().size(); ++i)
    {
      if (bGroup->button(i))
      {
        bGroup->removeButton(bGroup->button(i));

        delete bGroup->button(i);
      }
    }
  }

  if (layout)
  {
    clearLayout(layout);
    layout->update();

    d->UI.formLayout->update();
  }

  d->UI.groupBox->repaint();

  d->UI.formLayout->update();

  d->dcOptions.clear();
  d->gradients.clear();
}

//END DepthMapViewOptions
