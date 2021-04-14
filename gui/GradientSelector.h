/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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
