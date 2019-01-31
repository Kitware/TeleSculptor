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

#include "RulerHelper.h"

#include "CameraView.h"
#include "RulerWidget.h"
#include "MainWindow.h"
#include "WorldView.h"
#include "vtkMaptkCamera.h"
#include "vtkMaptkPointPicker.h"
#include "vtkMaptkPointPlacer.h"

#include <vital/types/geodesy.h>

#include <vital/range/transform.h>

#include <vtkHandleWidget.h>
#include <vtkPlane.h>
#include <vtkRenderer.h>

#include <QDebug>
#include <QMessageBox>

namespace kv = kwiver::vital;
namespace kvr = kwiver::vital::range;

//-----------------------------------------------------------------------------
class RulerHelperPrivate
{
public:
  RulerHelperPrivate(RulerHelper* q) : q_ptr{q} {}

  MainWindow* mainWindow = nullptr;

private:
  QTE_DECLARE_PUBLIC_PTR(RulerHelper)
  QTE_DECLARE_PUBLIC(RulerHelper)
};

QTE_IMPLEMENT_D_FUNC(RulerHelper)

//-----------------------------------------------------------------------------
RulerHelper::RulerHelper(QObject* parent)
  : QObject{parent}, d_ptr{new RulerHelperPrivate{this}}
{
  QTE_D();

  d->mainWindow = qobject_cast<MainWindow*>(parent);
  Q_ASSERT(d->mainWindow);

  RulerWidget* worldWidget =
    d->mainWindow->worldView()->rulerWidget();
  RulerWidget* cameraWidget =
    d->mainWindow->cameraView()->rulerWidget();

  // Set a point placer on the world widget.
  // This has to be set before the widget is enabled.
  worldWidget->setPointPlacer(vtkNew<vtkMaptkPointPlacer>());
}

//-----------------------------------------------------------------------------
RulerHelper::~RulerHelper()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::addCameraViewPoint()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::addWorldViewPoint()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::moveCameraViewPoint()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::moveWorldViewPoint()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::updateCameraViewRuler()
{
}

//-----------------------------------------------------------------------------
void RulerHelper::enableWidgets(bool enable)
{
  QTE_D();
  d->mainWindow->worldView()->rulerWidget()->enableWidget(
    enable);
  d->mainWindow->cameraView()->rulerWidget()->enableWidget(
    enable);
}
