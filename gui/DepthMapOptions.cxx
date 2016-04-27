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

//  QList<vtkActor*> actors;
  std::map<std::string, vtkProp3D*> actors;

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
          this, SLOT(switchsVisible(bool)));

  connect(d->UI.radioSurfaces, SIGNAL(toggled(bool)),
          this, SLOT(switchVisible(bool)));

  connect(d->UI.radioVertices, SIGNAL(toggled(bool)),
          this, SLOT(switchVisible(bool)));

}

DepthMapOptions::~DepthMapOptions()
{
  QTE_D();
  d->uiState.save();
}

void DepthMapOptions::addActor(std::string type, vtkProp3D* actor)
{
  QTE_D();

  d->actors.insert(std::pair<std::string, vtkProp3D*>(type,actor));

}

//void DepthMapOptions::switchPointsVisible(bool state)
//{
//  QTE_D();

//  d->actors[0]->SetVisibility(state);

//  emit this->depthMapChanged();
//}

//void DepthMapOptions::switchSurfacesVisible(bool state)
//{
//  QTE_D();
//  std::cout << "toggled" << std::endl;
//  d->actors[1]->SetVisibility(state);

//  emit this->depthMapChanged();
//}

void DepthMapOptions::switchVisible(bool state)
{
  QTE_D();

  if (d->UI.radioPoints->isEnabled())
    d->actors["points"]->SetVisibility(d->UI.radioPoints->isChecked());
  if (d->UI.radioSurfaces->isEnabled())
    d->actors["surfaces"]->SetVisibility(d->UI.radioSurfaces->isChecked());
  if (d->UI.radioVertices->isEnabled())
    d->actors["vertices"]->SetVisibility(d->UI.radioVertices->isChecked());

  emit this->depthMapChanged();

}

//void DepthMapOptions::enablePoints()
//{
//  QTE_D();

//  d->UI.radioPoints->setEnabled(true);
//  d->UI.radioPoints->setChecked(true);
//}

//void DepthMapOptions::enableSurfaces()
//{
//  QTE_D();

//  d->UI.radioSurfaces->setEnabled(true);

//  if(!d->UI.radioPoints->isEnabled())
//    d->UI.radioSurfaces->setChecked(true);
//}

void DepthMapOptions::enableDM(std::string type)
{
  QTE_D();
  if(type == "vtp")
  {
    d->UI.radioPoints->setEnabled(true);
    d->UI.radioPoints->setChecked(true);
  }
  else if(type == "vts")
  {
    d->UI.radioSurfaces->setEnabled(true);

    if(!d->UI.radioPoints->isEnabled())
      d->UI.radioSurfaces->setChecked(true);
  }
  else if(type == "vert")
  {
    d->UI.radioVertices->setEnabled(true);

    if(!d->UI.radioPoints->isEnabled())
      d->UI.radioVertices->setChecked(true);
  }
}

bool DepthMapOptions::isPointsChecked()
{
  QTE_D();

  return d->UI.radioPoints->isChecked();
}

bool DepthMapOptions::isSurfacesChecked()
{
  QTE_D();

  return d->UI.radioSurfaces->isChecked();
}

bool DepthMapOptions::isVerticesChecked()
{
  QTE_D();

  return d->UI.radioVertices->isChecked();
}

