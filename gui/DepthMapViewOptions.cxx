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

