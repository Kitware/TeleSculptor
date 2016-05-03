#include "VolumeOptions.h"
#include "ui_VolumeOptions.h"
#include "ColorizeSurfaceOptions.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QMenu>
#include <QtGui/QWidgetAction>
#include <QToolButton>

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkPointData.h>

//-----------------------------------------------------------------------------
class VolumeOptionsPrivate
{
public:
  VolumeOptionsPrivate() {}

  void setPopup(QToolButton* button, QWidget* popup);

  Ui::VolumeOptions UI;
  qtUiState uiState;
  ColorizeSurfaceOptions* colorizeSurfaceOptions;

  vtkActor* volumeActor;

};

QTE_IMPLEMENT_D_FUNC(VolumeOptions)

//-----------------------------------------------------------------------------
void VolumeOptionsPrivate::setPopup(QToolButton* button, QWidget* widget)
{
  auto const proxy = new QWidgetAction(button);
  proxy->setDefaultWidget(widget);

  auto const menu = new QMenu(button);
  menu->addAction(proxy);

  button->setMenu(menu);
}

//-----------------------------------------------------------------------------
VolumeOptions::VolumeOptions(const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new VolumeOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);


  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  d->colorizeSurfaceOptions = new ColorizeSurfaceOptions(settingsGroup, this);
  d->setPopup(d->UI.toolButtonColorizeSurfaceMenu, d->colorizeSurfaceOptions);
  // Connect signals/slots
  connect(d->UI.checkBoxColorizeSurface, SIGNAL(toggled(bool)),
          this, SLOT(showColorizeSurfaceMenu(bool)));

  connect(d->colorizeSurfaceOptions, SIGNAL(colorModeChanged(QString)),
          this, SLOT(updateColorizeSurfaceMenu(QString)));
}

//-----------------------------------------------------------------------------
VolumeOptions::~VolumeOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void VolumeOptions::setActor(vtkActor *actor)
{
  QTE_D();

  d->volumeActor = actor;

  vtkPointData* pointData = d->volumeActor->GetMapper()->GetInput()->GetPointData();
  int nbArray = pointData->GetNumberOfArrays();

  std::string name;

  for (int i = 0; i < nbArray; ++i) {

    name = pointData->GetArrayName(i);
    d->colorizeSurfaceOptions->addColorDisplay(name);
  }
}


//-----------------------------------------------------------------------------
void VolumeOptions::showColorizeSurfaceMenu(bool state)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setEnabled(state);
}

//-----------------------------------------------------------------------------
void VolumeOptions::updateColorizeSurfaceMenu(QString text)
{
  QTE_D();

  d->UI.toolButtonColorizeSurfaceMenu->setText(text);
}
