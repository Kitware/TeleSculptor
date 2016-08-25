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

#ifndef MAPTK_POINTOPTIONS_H_
#define MAPTK_POINTOPTIONS_H_

#include <qtGlobal.h>

#include <QtGui/QWidget>

class vtkActor;
class vtkMapper;

struct FieldInformation;

class PointOptionsPrivate;

class PointOptions : public QWidget
{
  Q_OBJECT

public:
  explicit PointOptions(QString const& settingsGroup,
                        QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~PointOptions();

  void addActor(vtkActor*);
  void addVisibleLandmarksActor(vtkActor*);
  void addNonVisibleLandmarksActor(vtkActor*);
  void addMapper(vtkMapper*);

  bool isVisibleLandmarksChecked();
  bool isVisibleLandmarksOnlyChecked();

  void setDefaultColor(QColor const&);

public slots:
  void setTrueColorAvailable(bool);
  void setDataFields(QHash<QString, FieldInformation> const&);

  void showVisibleLandmarksOnly(bool);
signals:
  void modified();
  void visibleLandmarksDisplayChanged(bool);
  void visibleLandmarksOnlyDisplayChanged(bool);

protected slots:
  void setSize(int);
  void setColorMode(int);

  void setDataColorIcon(QIcon const&);

  void updateActiveDataField();
  void updateFilters();

private:
  QTE_DECLARE_PRIVATE_RPTR(PointOptions)
  QTE_DECLARE_PRIVATE(PointOptions)

  QTE_DISABLE_COPY(PointOptions)
};

#endif
