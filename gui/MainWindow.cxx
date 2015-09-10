/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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

#include "MainWindow.h"

#include <maptk/camera_io.h>
#include <maptk/landmark_map_io.h>

#include <vtkAppendPolyData.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkFrustumSource.h>
#include <vtkNew.h>
#include <vtkPlanes.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>

#include <qtUiState.h>

#include <QApplication>
#include <QDebug>
#include <QFileDialog>

namespace // anonymous
{

//-----------------------------------------------------------------------------
void buildFrustum(vtkPlanes* out, maptk::vector_3d const& center,
                  maptk::vector_3d const& view, maptk::vector_3d const& up,
                  double fov, double aspectRatio)
{
  auto const depth = 15.0; // TODO make configurable or something
  auto const& focus = center + (view * depth / view.norm());

  vtkNew<vtkCamera> camera;

  camera->SetPosition(center[0], center[1], center[2]);
  camera->SetFocalPoint(focus[0], focus[1], focus[2]);
  camera->SetViewUp(up[0], up[1], up[2]);
//   camera->SetViewAngle(fov);
  camera->SetClippingRange(0.01, depth);

  double planeCoeffs[24];
  camera->GetFrustumPlanes(aspectRatio, planeCoeffs);
  out->SetFrustumPlanes(planeCoeffs);
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class MainWindowPrivate
{
public:
  Ui::MainWindow UI;
  qtUiState uiState;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  vtkNew<vtkRenderWindowInteractor> interactor;

  vtkNew<vtkAppendPolyData> cameraData;
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
MainWindow::MainWindow() : d_ptr(new MainWindowPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->uiState.mapState("Window/state", this);
  d->uiState.mapGeometry("Window/geometry", this);
  d->uiState.restore();

  connect(d->UI.actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));
  connect(d->UI.actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  // Set up render pipeline
  d->renderer->SetBackground(0, 0, 0);
  d->renderWindow->AddRenderer(d->renderer.GetPointer());
  d->UI.renderWidget->SetRenderWindow(d->renderWindow.GetPointer());

  // Set up actor for camera frustums
  vtkNew<vtkActor> cameraActor;
  vtkNew<vtkPolyDataMapper> cameraMapper;
  cameraMapper->SetInputConnection(d->cameraData->GetOutputPort());
  cameraActor->SetMapper(cameraMapper.GetPointer());
  cameraActor->GetProperty()->SetRepresentationToWireframe();

  d->renderer->AddActor(cameraActor.GetPointer());
}

//-----------------------------------------------------------------------------
MainWindow::~MainWindow()
{
  QTE_D();
  d->uiState.save();
}

//-----------------------------------------------------------------------------
void MainWindow::openFile()
{
  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open File", QString(),
    "All Supported Files (*.ply *.krtd);;"
    "PLY file (*.ply);;"
    "KRTD file (*.krtd);;"
    "All Files (*)");

  if (!paths.isEmpty())
  {
    this->openFiles(paths);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openFile(QString const& path)
{
  auto const fi = QFileInfo(path);
  if (fi.suffix().toLower() == "ply")
  {
    this->loadLandmarks(path);
  }
  else if (fi.suffix().toLower() == "krtd")
  {
    this->loadCamera(path);
  }
  else
  {
    qWarning() << "Don't know how to read file" << path
               << "(unrecognized extension)";
  }
}

//-----------------------------------------------------------------------------
void MainWindow::openFiles(QStringList const& paths)
{
  foreach (auto const& path, paths)
  {
    this->openFile(path);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadCamera(const QString& path)
{
  QTE_D();

  auto const& camera = maptk::read_krtd_file(qPrintable(path));

  // Get camera parameters
  auto const pixelAspect = camera.get_intrinsics().aspect_ratio();
  auto const focalLength = camera.get_intrinsics().focal_length();
  auto const aspect = pixelAspect; // TODO multiple by image aspect
  auto const fov = 10.0; // FIXME need to compute from image dimensions

  // Compute camera vectors from matrix
  auto const& center = camera.get_center();
  auto const& rotationMatrix =
    camera.get_rotation().quaternion().toRotationMatrix();
  auto const& up = -rotationMatrix.row(1).transpose();
  auto const& view = rotationMatrix.row(2).transpose();

  vtkNew<vtkPlanes> planes;
  buildFrustum(planes.GetPointer(), center, view, up, fov, aspect);

  vtkNew<vtkFrustumSource> frustum;
  frustum->SetPlanes(planes.GetPointer());
  frustum->SetShowLines(false);
  frustum->Update();

  d->cameraData->AddInputConnection(frustum->GetOutputPort());
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(const QString& path)
{
  QTE_D();

  auto const& landmarksPtr = maptk::read_ply_file(qPrintable(path));
  auto const& landmarks = landmarksPtr->landmarks();

  vtkNew<vtkPoints> points;
  vtkNew<vtkCellArray> verts;

  points->Allocate(static_cast<vtkIdType>(landmarks.size()));
  verts->Allocate(static_cast<vtkIdType>(landmarks.size()));

  vtkIdType vertIndex = 0;
  for (auto i = landmarks.cbegin(); i != landmarks.cend(); ++i)
  {
    auto const id = i->first;
    auto const& pos = i->second->loc();
    points->InsertNextPoint(pos.data());
    verts->InsertNextCell(1);
    verts->InsertCellPoint(vertIndex++);
  }

  vtkNew<vtkPolyData> polyData;
  vtkNew<vtkPolyDataMapper> mapper;

  polyData->SetPoints(points.GetPointer());
  polyData->SetVerts(verts.GetPointer());
  mapper->SetInputData(polyData.GetPointer());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper.GetPointer());
  actor->GetProperty()->SetPointSize(2);
  d->renderer->AddActor(actor.GetPointer());
}
