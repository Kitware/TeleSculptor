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

#include <vtkGlyph3D.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>

#include <QApplication>
#include <QDebug>
#include <QFileDialog>

//-----------------------------------------------------------------------------
class MainWindow::Private
{
public:
  Ui::MainWindow UI;

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  vtkNew<vtkRenderWindowInteractor> interactor;
};

//-----------------------------------------------------------------------------
MainWindow::MainWindow() : d(new Private)
{
  // Set up UI
  this->d->UI.setupUi(this);

  connect(this->d->UI.actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));
  connect(this->d->UI.actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  // Set up render pipeline
  this->d->renderer->SetBackground(0, 0, 0);
  this->d->renderWindow->AddRenderer(this->d->renderer.GetPointer());
  this->d->UI.renderWidget->SetRenderWindow(this->d->renderWindow.GetPointer());
}

//-----------------------------------------------------------------------------
MainWindow::~MainWindow()
{
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
  auto const& camera = maptk::read_krtd_file(qPrintable(path));
  // TODO
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(const QString& path)
{
  auto const& landmarksPtr = maptk::read_ply_file(qPrintable(path));
  auto const& landmarks = landmarksPtr->landmarks();

  vtkNew<vtkPoints> points;

  points->SetNumberOfPoints(static_cast<vtkIdType>(landmarks.size()));

  for (auto i = landmarks.cbegin(); i != landmarks.cend(); ++i)
  {
    auto const id = i->first;
    auto const& pos = i->second->loc();
    points->InsertNextPoint(pos.data());
  }

  vtkNew<vtkGlyph3D> glyph;
  vtkNew<vtkSphereSource> sphere;

  vtkNew<vtkPolyData> polyData;
  vtkNew<vtkPolyDataMapper> mapper;

  sphere->SetRadius(0.01);

  polyData->SetPoints(points.GetPointer());
  glyph->SetInputData(polyData.GetPointer());
  glyph->SetSourceConnection(sphere->GetOutputPort());
  mapper->SetInputConnection(glyph->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper.GetPointer());
  this->d->renderer->AddActor(actor.GetPointer());
}
