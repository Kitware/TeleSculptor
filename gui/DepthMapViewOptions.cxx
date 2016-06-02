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

#include <vtkScalarsToColors.h>
#include <vtkImageMapper3D.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToColors.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>


//-----------------------------------------------------------------------------
class DepthMapViewOptionsPrivate
{
public:
  Ui::DepthMapViewOptions UI;
  qtUiState uiState;

  vtkImageActor* imageActor;
  vtkSmartPointer<vtkImageData> originalImage;

  std::map<std::string, DataColorOptions*> dcOptions;

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

  layout = new QVBoxLayout(d->UI.groupBox);
  d->UI.groupBox->setLayout(layout);
  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots

  d->originalImage = vtkSmartPointer<vtkImageData>::New();

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

  int index = bGroup->buttons().size();
  bGroup->addButton(scalar,index);

  if (needGradient)
  {
    QToolButton *gradient = new QToolButton(d->UI.groupBox);
    DataColorOptions *dataColorOptions = new DataColorOptions("DepthMapViewOptions/"+QString::fromStdString(name),
                                                              this);
    dataColorOptions->setEnabled(false);
    d->setPopup(gradient, dataColorOptions);
    QHBoxLayout *hLayout = new QHBoxLayout();
    hLayout->addWidget(scalar);
    hLayout->addWidget(gradient);
    layout->addLayout(hLayout);
    gradient->setIcon(dataColorOptions->icon());

    dataColorOptions->setEnabled(true);

//    connect(dataColorOptions, SIGNAL(iconChanged(QIcon)),
//            gradient, SLOT());

    d->dcOptions.insert(std::pair<std::string, DataColorOptions*>(name, dataColorOptions));
  }
  else
  {
    layout->addWidget(scalar);
  }

  scalar->setVisible(true);
  scalar->setEnabled(true);
  scalar->setCheckable(true);



//  connect(dataColorOptions, SIGNAL(modified()), this, SIGNAL(modified()));
  connect(scalar, SIGNAL(toggled(bool)),
          this, SLOT(switchDisplayMode(bool)));

}

void DepthMapViewOptions::switchDisplayMode(bool checked)
{
  QTE_D();

  //This way it's only triggered on the checked event and not on the unchecked too
  if (checked)
  {
    std::string buttonId = bGroup->checkedButton()->text().toStdString();
    std::cout << "buttonId = " << buttonId <<std::endl;


    vtkNew<vtkImageData> im;
    im->DeepCopy(d->originalImage);
    d->imageActor->SetInputData(im.Get());

    d->imageActor->GetInput()->GetPointData()
        ->SetActiveScalars(buttonId.c_str());

    //Displaying the scalar array associated with the checked radio button
    if (d->imageActor->GetInput()->GetPointData()->GetArray(buttonId.c_str())->GetNumberOfComponents() < 3)
    {
      DataColorOptions *dc = d->dcOptions.at(buttonId);

      vtkNew<vtkImageMapToColors> imageMapToColors;
      imageMapToColors->SetLookupTable(dc->scalarsToColors());
      imageMapToColors->PassAlphaToOutputOn();
      imageMapToColors->SetInputData(d->imageActor->GetInput());

      imageMapToColors->Update();

      d->imageActor->SetInputData(imageMapToColors->GetOutput());
    }

    emit this->modified();
  }
}

void DepthMapViewOptions::addActor(vtkImageActor *actor)
{
  QTE_D();

  cleanModes();
  bGroup = new QButtonGroup(d->UI.groupBox);
  bool needGradient;
  d->imageActor = actor;
  d->originalImage->DeepCopy(d->imageActor->GetInput());

  for (int i = 0; i < d->imageActor->GetInput()->GetPointData()->GetNumberOfArrays(); ++i)
  {
    std::cout << d->imageActor->GetInput()->GetPointData()->GetArray(i) << std::endl;
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

  bGroup->buttons()[1]->setChecked(true);

}

void DepthMapViewOptions::removeLayout(QLayout* layout)
{
    QLayoutItem* child;
    while(layout->count()!=0)
    {
        child = layout->takeAt(0);
        if(child->layout() != 0)
        {
            removeLayout(child->layout());
        }
        else if(child->widget() != 0)
        {
            delete child->widget();
        }

        delete child;
    }
}

void DepthMapViewOptions::cleanModes()
{
  QTE_D();

  if (layout)
  {
    removeLayout(layout);
    layout->update();
    d->UI.formLayout->update();
  }

  if(bGroup)
  {

    for (int i = 0; i < bGroup->buttons().size(); ++i)
    {
      if (bGroup->button(i))
      {
        bGroup->removeButton(bGroup->button(i));
      }
    }
  }


  d->UI.groupBox->repaint();

  d->UI.formLayout->update();

  d->dcOptions.clear();
}

