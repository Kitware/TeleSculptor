// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
    case GradientSelector::Slate:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.25, QColor(56, 56, 76));
      gradient.insertStop(0.37, QColor(84, 84, 112));
      gradient.insertStop(0.50, QColor(112, 120, 144));
      gradient.insertStop(0.75, QColor(172, 200, 200));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Wood:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.25, QColor(104, 64, 64));
      gradient.insertStop(0.50, QColor(192, 128, 88));
      gradient.insertStop(0.75, QColor(224, 208, 136));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

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

    case GradientSelector::Sky:
      gradient.insertStop(0.00, QColor(8, 48, 108));
      gradient.insertStop(0.25, QColor(32, 112, 180));
      gradient.insertStop(0.50, QColor(104, 172, 212));
      gradient.insertStop(0.75, QColor(196, 216, 236));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Hyacinth:
      gradient.insertStop(0.00, QColor(76, 0, 76));
      gradient.insertStop(0.15, QColor(120, 20, 120));
      gradient.insertStop(0.25, QColor(136, 64, 156));
      gradient.insertStop(0.50, QColor(140, 148, 196));
      gradient.insertStop(0.75, QColor(188, 212, 228));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Lilac:
      gradient.insertStop(0.00, QColor(104, 0, 40));
      gradient.insertStop(0.25, QColor(204, 16, 96));
      gradient.insertStop(0.33, QColor(216, 48, 132));
      gradient.insertStop(0.50, QColor(216, 96, 168));
      gradient.insertStop(0.62, QColor(200, 144, 196));
      gradient.insertStop(0.75, QColor(212, 184, 216));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Jet:
      gradient.insertStop(0.00, QColor(0, 0, 128));
      gradient.insertStop(0.15, QColor(0, 0, 255));
      gradient.insertStop(0.35, QColor(0, 255, 255));
      gradient.insertStop(0.65, QColor(255, 255, 0));
      gradient.insertStop(0.85, QColor(255, 0, 0));
      gradient.insertStop(1.00, QColor(128, 0, 0));
      return gradient;

    case GradientSelector::Terrain:
      gradient.insertStop(0.00, QColor(40, 40, 128));
      gradient.insertStop(0.07, QColor(20, 80, 160));
      gradient.insertStop(0.15, QColor(0, 120, 192));
      gradient.insertStop(0.25, QColor(0, 184, 92));
      gradient.insertStop(0.38, QColor(108, 220, 120));
      gradient.insertStop(0.50, QColor(252, 255, 152));
      gradient.insertStop(0.62, QColor(192, 160, 120));
      gradient.insertStop(0.75, QColor(128, 92, 84));
      gradient.insertStop(0.88, QColor(192, 164, 160));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      gradient.setInterpolationMode(qtGradient::InterpolateCubic);
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

    case GradientSelector::Plasma:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.08, QColor(0, 16, 80));
      gradient.insertStop(0.15, QColor(40, 0, 144));
      gradient.insertStop(0.35, QColor(160, 0, 144));
      gradient.insertStop(0.55, QColor(232, 60, 36));
      gradient.insertStop(0.75, QColor(255, 232, 48));
      gradient.insertStop(1.00, QColor(255, 255, 255));
      return gradient;

    case GradientSelector::Blackbody:
      gradient.insertStop(0.00, QColor(0, 0, 0));
      gradient.insertStop(0.15, QColor(152, 24, 0));
      gradient.insertStop(0.27, QColor(232, 44, 0));
      gradient.insertStop(0.40, QColor(255, 176, 40));
      gradient.insertStop(0.55, QColor(255, 232, 126));
      gradient.insertStop(0.75, QColor(255, 255, 255));
      gradient.insertStop(1.00, QColor(144, 172, 255));
      return gradient;

    case GradientSelector::Spring:
      gradient.insertStop(0.00, Qt::yellow);
      gradient.insertStop(1.00, Qt::cyan);
      return gradient;

    case GradientSelector::Tulip:
      gradient.insertStop(0.00, Qt::magenta);
      gradient.insertStop(1.00, Qt::yellow);
      return gradient;

    case GradientSelector::Moody:
      gradient.insertStop(0.00, Qt::cyan);
      gradient.insertStop(1.00, Qt::magenta);
      return gradient;

    case GradientSelector::Anaglyph:
      gradient.insertStop(0.00, QColor(0, 96, 192));
      gradient.insertStop(1.00, QColor(176, 0, 48));
      return gradient;

    case GradientSelector::Rainbow:
      gradient.insertStop(0.0 / 6.0, Qt::red);
      gradient.insertStop(1.0 / 6.0, Qt::yellow);
      gradient.insertStop(2.0 / 6.0, Qt::green);
      gradient.insertStop(3.0 / 6.0, Qt::cyan);
      gradient.insertStop(4.0 / 6.0, Qt::blue);
      gradient.insertStop(5.0 / 6.0, Qt::magenta);
      gradient.insertStop(6.0 / 6.0, Qt::red);
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
  addGradient(this, Slate, "Slate");
  addGradient(this, Wood, "Wood");
  addGradient(this, Royal, "Royal");
  addGradient(this, Ember, "Ember");
  addGradient(this, Ocean, "Ocean");
  addGradient(this, Sunset, "Sunset");
  addGradient(this, Carribean, "Carribean");
  addGradient(this, Sky, "Sky");
  addGradient(this, Hyacinth, "Hyacinth");
  addGradient(this, Lilac, "Lilac");
  addGradient(this, Jet, "Jet");
  addGradient(this, Terrain, "Terrain");
  addGradient(this, Parula, "Parula");
  addGradient(this, Viridis, "Viridis");
  addGradient(this, Earth, "Earth");
  addGradient(this, Plasma, "Plasma");
  addGradient(this, Blackbody, "Blackbody");
  addGradient(this, Spring, "Spring");
  addGradient(this, Tulip, "Tulip");
  addGradient(this, Moody, "Moody");
  addGradient(this, Anaglyph, "Anaglyph");
  addGradient(this, Rainbow, "Rainbow");
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
