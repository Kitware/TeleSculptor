#include "DepthMapOptions.h"
#include "ui_DepthMapOptions.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <vtkActor.h>

//-----------------------------------------------------------------------------
class DepthMapOptionsPrivate
{
public:
  Ui::DepthMapOptions UI;
  qtUiState uiState;

  QList<vtkActor*> actors;

};

QTE_IMPLEMENT_D_FUNC(DepthMapOptions)

DepthMapOptions::DepthMapOptions(QString const& settingsGroup,
                                 QWidget* parent, Qt::WindowFlags flags) :
  QWidget(parent, flags), d_ptr(new DepthMapOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.radioPoints, SIGNAL(toggled(bool)),
          this, SLOT(switchPointsVisible(bool)));

  connect(d->UI.radioSurfaces, SIGNAL(toggled(bool)),
          this, SLOT(switchSurfacesVisible(bool)));
}

DepthMapOptions::~DepthMapOptions()
{
  QTE_D();
  d->uiState.save();
}

void DepthMapOptions::addActor(vtkActor* actor)
{
  QTE_D();

  d->actors.append(actor);
}

void DepthMapOptions::switchPointsVisible(bool state)
{
  QTE_D();

  d->actors[0]->SetVisibility(state);

  emit this->depthMapChanged();
}

void DepthMapOptions::switchSurfacesVisible(bool state)
{
  QTE_D();
  std::cout << "toggled" << std::endl;
  d->actors[1]->SetVisibility(state);

  emit this->depthMapChanged();
}

void DepthMapOptions::enablePoints()
{
  QTE_D();

  d->UI.radioPoints->setEnabled(true);
  d->UI.radioPoints->setChecked(true);
}

void DepthMapOptions::enableSurfaces()
{
  QTE_D();

  d->UI.radioSurfaces->setEnabled(true);

  if(!d->UI.radioPoints->isEnabled())
    d->UI.radioSurfaces->setChecked(true);
}

bool DepthMapOptions::isPointsChecked()
{
  QTE_D();

//  emit this->depthMapChanged();
  return d->UI.radioPoints->isChecked();
}

bool DepthMapOptions::isSurfacesChecked()
{
  QTE_D();

//  emit this->depthMapChanged();
  return d->UI.radioSurfaces->isChecked();
}
