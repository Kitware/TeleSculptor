#include "ColorizeSurfaceOptions.h"
#include "ui_ColorizeSurfaceOptions.h"

#include <qtUiState.h>
#include <qtUiStateItem.h>

//-----------------------------------------------------------------------------
class ColorizeSurfaceOptionsPrivate
{
public:
  ColorizeSurfaceOptionsPrivate() {}

  Ui::ColorizeSurfaceOptions UI;
  qtUiState uiState;

};

QTE_IMPLEMENT_D_FUNC(ColorizeSurfaceOptions)

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::ColorizeSurfaceOptions(const QString &settingsGroup, QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new ColorizeSurfaceOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);


  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.radioButtonCurrentFrame, SIGNAL(toggled(bool)),
          this, SLOT(toggleAllFramesMenu()));

  connect(d->UI.radioButtonAllFrames, SIGNAL(toggled(bool)),
          this, SLOT(toggleAllFramesMenu()));
}

//-----------------------------------------------------------------------------
ColorizeSurfaceOptions::~ColorizeSurfaceOptions()
{
  QTE_D();
  d->uiState.save();
}

void ColorizeSurfaceOptions::addColorDisplay(std::string name)
{
  QTE_D();

  d->UI.comboBoxColorDisplay->addItem(QString(name.c_str()));
}

//-----------------------------------------------------------------------------
void ColorizeSurfaceOptions::toggleAllFramesMenu()
{
  QTE_D();
  bool state = d->UI.radioButtonAllFrames->isChecked();

  d->UI.buttonCompute->setEnabled(state);
  d->UI.comboBoxColorDisplay->setEnabled(state);
  d->UI.spinBoxFrameSampling->setEnabled(state);

  emit colorModeChanged(d->UI.buttonGroup->checkedButton()->text());
}
