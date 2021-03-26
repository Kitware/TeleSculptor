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

#ifndef TELESCULPTOR_DEPTHMAPOPTIONS_H_
#define TELESCULPTOR_DEPTHMAPOPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class vtkProp3D;

class DepthMapOptionsPrivate;

class DepthMapOptions : public QWidget
{
  Q_OBJECT

public:
  enum DisplayMode
  {
    Points,
    Surfaces,
  };

public:
  explicit DepthMapOptions(QString const& settingsGroup,
                           QWidget* parent = nullptr,
                           Qt::WindowFlags flags = {});
  ~DepthMapOptions() override;

  DisplayMode displayMode() const;

  double weightMinimum() const;
  double weightMaximum() const;
  double uncertaintyMinimum() const;
  double uncertaintyMaximum() const;

  void initializeFilters(double bcMin, double bcMax,
                         double urMin, double urMax);

  bool isFilterPersistent() const;
  bool isFilterEnabled() const;

signals:
  void displayModeChanged();
  void thresholdsChanged(bool);

protected slots:
  void thresholdsChanged();

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapOptions)
  QTE_DECLARE_PRIVATE(DepthMapOptions)

  QTE_DISABLE_COPY(DepthMapOptions)
};

#endif
