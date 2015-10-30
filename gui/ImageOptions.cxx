/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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
  connect(d->UI.opacity, SIGNAL(valueChanged(double)),
          this, SLOT(setOpacity(double)));
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
