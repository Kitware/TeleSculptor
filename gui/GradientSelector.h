// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_GRADIENTSELECTOR_H_
#define TELESCULPTOR_GRADIENTSELECTOR_H_

#include <qtGlobal.h>

#include <QComboBox>

class qtGradient;

class GradientSelector : public QComboBox
{
  Q_OBJECT

public:
  enum Preset
  {
    Gray,
    Slate,
    Wood,
    Royal,
    Ember,
    Ocean,
    Sunset,
    Carribean,
    Sky,
    Hyacinth,
    Lilac,
    // TODO divergent gradients
    Jet,
    Terrain,
    Parula,
    Viridis,
    Earth,
    Plasma,
    Blackbody,
    Spring,
    Tulip,
    Moody,
    Anaglyph,
    Rainbow
  };

  explicit GradientSelector(QWidget* parent = nullptr);
  ~GradientSelector() override;

  qtGradient currentGradient() const;
  qtGradient gradient(Preset) const;

private:
  QTE_DISABLE_COPY(GradientSelector)
};

#endif
