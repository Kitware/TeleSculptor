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

//  auto const opacityItem = new qtUiState::Item<double, qtDoubleSlider>(
//    d->UI.opacity, &qtDoubleSlider::value, &qtDoubleSlider::setValue);
//  d->uiState.map("Opacity", opacityItem);

  d->uiState.restore();

  // Connect signals/slots
//  connect(d->UI.opacity, SIGNAL(valueChanged(double)),
//          this, SLOT(setOpacity(double)));
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
