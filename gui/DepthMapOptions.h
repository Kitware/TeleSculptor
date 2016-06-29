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

#ifndef DEPTHMAPOPTIONS_H
#define DEPTHMAPOPTIONS_H

#include <qtGlobal.h>

#include <QtGui/QWidget>

class vtkProp3D;

class DepthMapOptionsPrivate;

class DepthMapOptions : public QWidget
{
  Q_OBJECT

public:
  explicit DepthMapOptions(const QString &settingsGroup,
                           QWidget *parent = 0, Qt::WindowFlags flags = 0);
  virtual ~DepthMapOptions();

  void addActor(std::string type, vtkProp3D *actor);

  bool isPointsChecked();
  bool isSurfacesChecked();
  bool isVerticesChecked();

  void enableDM(std::string type);
//  void enablePoints();
//  void enableSurfaces();
//  void enableVertices();

signals:
  void modified();
  void depthMapChanged();

protected slots:
//  void switchPointsVisible(bool);
//  void switchSurfacesVisible(bool);
  void switchVisible(bool);

private:
  QTE_DECLARE_PRIVATE_RPTR(DepthMapOptions)
  QTE_DECLARE_PRIVATE(DepthMapOptions)

  QTE_DISABLE_COPY(DepthMapOptions)
};

#endif // DEPTHMAPOPTIONS_H
