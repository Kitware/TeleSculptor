/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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

#ifndef TELESCULPTOR_RULERHELPER_H_
#define TELESCULPTOR_RULERHELPER_H_

// qtExtensions includes
#include <qtGlobal.h>

// Qt declarations
#include <QColor>
#include <QObject>

// Forward declarations
class RulerHelperPrivate;
class RulerWidget;

class RulerHelper : public QObject
{
  Q_OBJECT

public:
  RulerHelper(QObject* parent = nullptr);
  ~RulerHelper();

  // Update the camera view ruler from the world view
  void updateCameraViewRuler();

  RulerWidget* worldWidget();
  RulerWidget* cameraWidget();

public slots:
  void enableWidgets(bool);
  void resetRuler();
  void setRulerTickDistance(double scale);
  void setRulerColor(const QColor& rgb);

protected slots:
  void addWorldViewPoint(int pId);
  void addCameraViewPoint(int pId);

  void moveCameraViewPoint(int pId);
  void moveWorldViewPoint(int pId);

private:
  QTE_DECLARE_PRIVATE_RPTR(RulerHelper)
  QTE_DECLARE_PRIVATE(RulerHelper)

  QTE_DISABLE_COPY(RulerHelper)
};

#endif
