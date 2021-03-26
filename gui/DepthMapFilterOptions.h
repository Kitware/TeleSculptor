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

#ifndef TELESCULPTOR_DEPTHMAPFILTEROPTIONS_H_
#define TELESCULPTOR_DEPTHMAPFILTEROPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class DepthMapFilterOptionsPrivate;

class DepthMapFilterOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapFilterOptions(QString const& settingsGroup,
                                 QWidget* parent = nullptr,
                                 Qt::WindowFlags flags = {});
  ~DepthMapFilterOptions() override;

  double weightMinimum() const;
  double weightMaximum() const;
  double uncertaintyMinimum() const;
  double uncertaintyMaximum() const;

  void initializeFilters(double wMin, double wMax,
                         double uMin, double uMax);

  bool isFilterPersistent() const;

signals:
  void filtersChanged();

public slots:
  void updateWeightMinimum();
  void updateWeightMaximum();
  void updateUncertaintyMinimum();
  void updateUncertaintyMaximum();

  void resetFilters();

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapFilterOptions)
  QTE_DECLARE_PRIVATE(DepthMapFilterOptions)

  QTE_DISABLE_COPY(DepthMapFilterOptions)
};

#endif
