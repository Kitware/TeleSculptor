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

#include "GradientSelector.h"

#include <qtGradient.h>
#include <qtIndexRange.h>

///////////////////////////////////////////////////////////////////////////////

//BEGIN helper functions

namespace // anonymous
{

//-----------------------------------------------------------------------------
qtGradient buildGradient(GradientSelector::Preset which)
{
  auto gradient = qtGradient{};

  switch (which)
  {
    case GradientSelector::Royal:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.25, QColor(64, 32, 120));
      gradient.insertStop(0.55, QColor(160, 112, 112));
      gradient.insertStop(0.85, QColor(255, 232, 128));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Ember:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.25, QColor(128, 0, 0));
      gradient.insertStop(0.50, QColor(255, 128, 0));
      gradient.insertStop(0.75, QColor(255, 255, 128));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Ocean:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.15, QColor(0, 24, 92));
      gradient.insertStop(0.30, QColor(0, 56, 120));
      gradient.insertStop(0.50, QColor(0, 112, 160));
      gradient.insertStop(0.75, QColor(120, 188, 212));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Sunset:
      gradient.insertStop(0.00, QColor(16, 56, 126));
      gradient.insertStop(0.25, QColor(116, 72, 176));
      gradient.insertStop(0.50, QColor(184, 116, 160));
      gradient.insertStop(0.70, QColor(224, 156, 144));
      gradient.insertStop(0.85, QColor(240, 200, 144));
      gradient.insertStop(1.00, QColor(244, 244, 184));
      return gradient;

    case GradientSelector::Carribean:
      gradient.insertStop(0.00, QColor(0, 32, 96));
      gradient.insertStop(0.15, QColor(32, 60, 152));
      gradient.insertStop(0.30, QColor(32, 112, 176));
      gradient.insertStop(0.60, QColor(88, 192, 192));
      gradient.insertStop(0.80, QColor(216, 240, 192));
      gradient.insertStop(1.00, QColor(255, 255, 216));
      return gradient;

    case GradientSelector::Jet:
      gradient.insertStop(0.00, QColor(0, 0, 128));
      gradient.insertStop(0.15, QColor(0, 0, 255));
      gradient.insertStop(0.35, QColor(0, 255, 255));
      gradient.insertStop(0.65, QColor(255, 255, 0));
      gradient.insertStop(0.85, QColor(255, 0, 0));
      gradient.insertStop(1.00, QColor(128, 0, 0));
      return gradient;

    case GradientSelector::Parula:
      gradient.insertStop(0.00, QColor(52, 44, 140));
      gradient.insertStop(0.05, QColor(52, 60, 168));
      gradient.insertStop(0.10, QColor(28, 84, 212));
      gradient.insertStop(0.15, QColor(0, 104, 224));
      gradient.insertStop(0.25, QColor(20, 128, 212));
      gradient.insertStop(0.35, QColor(4, 160, 204));
      gradient.insertStop(0.50, QColor(48, 184, 160));
      gradient.insertStop(0.60, QColor(120, 192, 124));
      gradient.insertStop(0.75, QColor(212, 184, 88));
      gradient.insertStop(0.85, QColor(255, 192, 60));
      gradient.insertStop(1.00, QColor(248, 248, 16));
      return gradient;

    case GradientSelector::Viridis:
      gradient.insertStop(0.00, QColor(68, 0, 84));
      gradient.insertStop(0.15, QColor(68, 52, 128));
      gradient.insertStop(0.30, QColor(52, 96, 140));
      gradient.insertStop(0.50, QColor(32, 144, 140));
      gradient.insertStop(0.65, QColor(44, 176, 124));
      gradient.insertStop(0.80, QColor(120, 208, 80));
      gradient.insertStop(1.00, QColor(255, 232, 36));
      return gradient;

    case GradientSelector::Earth:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.02, QColor(3, 0, 88));
      gradient.insertStop(0.05, QColor(7, 10, 116));
      gradient.insertStop(0.15, QColor(24, 68, 120));
      gradient.insertStop(0.25, QColor(40, 114, 125));
      gradient.insertStop(0.30, QColor(48, 130, 121));
      gradient.insertStop(0.40, QColor(62, 145, 89));
      gradient.insertStop(0.45, QColor(68, 152, 73));
      gradient.insertStop(0.50, QColor(94, 160, 75));
      gradient.insertStop(0.60, QColor(142, 171, 86));
      gradient.insertStop(0.65, QColor(164, 178, 90));
      gradient.insertStop(0.70, QColor(183, 181, 94));
      gradient.insertStop(0.75, QColor(189, 169, 98));
      gradient.insertStop(0.80, QColor(197, 164, 114));
      gradient.insertStop(0.85, QColor(212, 176, 147));
      gradient.insertStop(0.90, QColor(227, 195, 182));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Blackbody:
      gradient.insertStop(0.000, QColor(0, 0, 0));
      gradient.insertStop(0.150, QColor(152, 24, 0));
      gradient.insertStop(0.270, QColor(232, 44, 0));
      gradient.insertStop(0.400, QColor(255, 176, 40));
      gradient.insertStop(0.550, QColor(255, 232, 126));
      gradient.insertStop(0.750, QColor(255, 255, 255));
      gradient.insertStop(1.000, QColor(144, 172, 255));
      return gradient;

    default: // Gray
      return gradient;
  }
}

//-----------------------------------------------------------------------------
QImage gradientPreview(qtGradient const& gradient, int w = 64, int h = 12)
{
  auto image = QImage(w, h, QImage::Format_ARGB32);
  auto const s = 1.0 / (static_cast<double>(w) - 1.0);

  foreach (auto const i, qtIndexRange(w))
  {
    auto const a = static_cast<double>(i) * s;
    auto const c = gradient.at(a).rgba();

    foreach (auto const j, qtIndexRange(h))
    {
      image.setPixel(i, j, c);
    }
  }

  return image;
}

//-----------------------------------------------------------------------------
void addGradient(
  QComboBox* box, GradientSelector::Preset preset, QString const& name)
{
  auto const& gradient = buildGradient(preset);
  auto const& preview = QPixmap::fromImage(gradientPreview(gradient));

  auto const i = box->count();
  box->addItem(name);
  box->setItemData(i, QIcon(preview), Qt::DecorationRole);
}

} // namespace <anonymous>

//END helper functions

///////////////////////////////////////////////////////////////////////////////

//BEGIN GradientSelector

//-----------------------------------------------------------------------------
GradientSelector::GradientSelector(QWidget* parent) : QComboBox{parent}
{
  addGradient(this, Gray, "Gray");
  addGradient(this, Royal, "Royal");
  addGradient(this, Ember, "Ember");
  addGradient(this, Ocean, "Ocean");
  addGradient(this, Sunset, "Sunset");
  addGradient(this, Carribean, "Carribean");
  addGradient(this, Jet, "Jet");
  addGradient(this, Parula, "Parula");
  addGradient(this, Viridis, "Viridis");
  addGradient(this, Earth, "Earth");
  addGradient(this, Blackbody, "Blackbody");
}

//-----------------------------------------------------------------------------
GradientSelector::~GradientSelector()
{
}

//-----------------------------------------------------------------------------
qtGradient GradientSelector::gradient(Preset which) const
{
  return buildGradient(which);
}

//-----------------------------------------------------------------------------
qtGradient GradientSelector::currentGradient() const
{
  return this->gradient(static_cast<Preset>(this->currentIndex()));
}

//END GradientSelector
