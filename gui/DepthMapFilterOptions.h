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

#ifndef MAPTK_DEPTHMAPFILTEROPTIONS_H_
#define MAPTK_DEPTHMAPFILTEROPTIONS_H_

#include <qtGlobal.h>

#include <QWidget>

class DepthMapFilterOptionsPrivate;

class DepthMapFilterOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapFilterOptions(const QString& settingsGroup,
                                 QWidget* parent = 0,
                                 Qt::WindowFlags flags = 0);
  virtual ~DepthMapFilterOptions();

  double bestCostValueMinimum() const;
  double bestCostValueMaximum() const;
  double uniquenessRatioMinimum() const;
  double uniquenessRatioMaximum() const;

  void initializeFilters(double bcMin, double bcMax,
                         double urMin, double urMax);

  bool isFilterPersistent() const;

signals:
  void filtersChanged();

public slots:
  void updateBestCostMinimum();
  void updateBestCostMaximum();
  void updateUniquenessRatioMinimum();
  void updateUniquenessRatioMaximum();

  void resetFilters();

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapFilterOptions)
  QTE_DECLARE_PRIVATE(DepthMapFilterOptions)

  QTE_DISABLE_COPY(DepthMapFilterOptions)
};

#endif
