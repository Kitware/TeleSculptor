// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "ImageOptions.h"

#include "ui_ImageOptions.h"

#include <vtkImageActor.h>

#include <qtUiState.h>
#include <qtUiStateItem.h>

//-----------------------------------------------------------------------------
class ImageOptionsPrivate
{
public:
  Ui::ImageOptions UI;
  qtUiState uiState;

  QList<vtkImageActor*> actors;
};

QTE_IMPLEMENT_D_FUNC(ImageOptions)

//-----------------------------------------------------------------------------
ImageOptions::ImageOptions(QString const& settingsGroup,
                           QWidget* parent, Qt::WindowFlags flags)
  : QWidget(parent, flags), d_ptr(new ImageOptionsPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  // Set up option persistence
  d->uiState.setCurrentGroup(settingsGroup);

  auto const opacityItem = new qtUiState::Item<double, qtDoubleSlider>(
    d->UI.opacity, &qtDoubleSlider::value, &qtDoubleSlider::setValue);
  d->uiState.map("Opacity", opacityItem);

  d->uiState.restore();

  // Connect signals/slots
  connect(d->UI.opacity, &qtDoubleSlider::valueChanged,
          this, &ImageOptions::setOpacity);
}

//-----------------------------------------------------------------------------
ImageOptions::~ImageOptions()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void ImageOptions::addActor(vtkImageActor* actor)
{
  QTE_D();

  actor->SetOpacity(d->UI.opacity->value());
  d->actors.append(actor);
}

//-----------------------------------------------------------------------------
void ImageOptions::setOpacity(double opacity)
{
  QTE_D();

  foreach (auto const actor, d->actors)
  {
    actor->SetOpacity(opacity);
  }

  emit this->modified();
}
