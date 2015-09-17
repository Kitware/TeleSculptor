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

#include "Project.h"
#include "vtkMkCamera.h"

#include <maptk/camera_io.h>
#include <maptk/landmark_map_io.h>

#include <qtUiState.h>

#include <QtGui/QApplication>
#include <QtGui/QFileDialog>

#include <QtCore/QDebug>
#include <QtCore/QTimer>

namespace // anonymous
{

//-----------------------------------------------------------------------------
struct CameraData
{
  int id;
  vtkMkCamera* camera;

  QString imagePath; // Full path to camera image data
};

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class MainWindowPrivate
{
public:
  MainWindowPrivate(MainWindow* q) : q_ptr(q) {}

  ~MainWindowPrivate()
  {
    foreach(auto camera, cameras)
    {
      camera.camera->Delete();
    }
  }

  void addCamera(CameraData&, QString const&);

  Ui::MainWindow UI;
  qtUiState uiState;

  QTimer slideTimer;

  QList<CameraData> cameras;
  maptk::landmark_map_sptr landmarks;

private:
  QTE_DECLARE_PUBLIC_PTR(MainWindow)
  QTE_DECLARE_PUBLIC(MainWindow)
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
void MainWindowPrivate::addCamera(CameraData& cd, QString const& cameraPath)
{
  QTE_Q();

  cd.id = this->cameras.count();

  cd.camera = vtkMkCamera::New();
  cd.camera->SetCamera(maptk::read_krtd_file(qPrintable(cameraPath)));
  cd.camera->Update();

  this->cameras.append(cd);

  this->UI.worldView->addCamera(cd.id, cd.camera);

  this->UI.actionSlideshowPlay->setEnabled(true);
  this->UI.camera->setEnabled(true);
  this->UI.camera->setRange(0, this->cameras.count() - 1);

  if (this->cameras.count() == 1)
  {
    q->setActiveCamera(0);
  }
}

//-----------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags), d_ptr(new MainWindowPrivate(this))
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);

  d->UI.menuView->addSeparator();
  d->UI.menuView->addAction(d->UI.cameraViewDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.cameraSelectorDock->toggleViewAction());

  d->UI.playSlideshowButton->setDefaultAction(d->UI.actionSlideshowPlay);
  d->UI.loopSlideshowButton->setDefaultAction(d->UI.actionSlideshowLoop);

  connect(d->UI.actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));
  connect(d->UI.actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  connect(&d->slideTimer, SIGNAL(timeout()), this, SLOT(nextSlide()));
  connect(d->UI.actionSlideshowPlay, SIGNAL(toggled(bool)),
          this, SLOT(setSlideshowPlaying(bool)));
  connect(d->UI.slideDelay, SIGNAL(valueChanged(int)),
          this, SLOT(setSlideDelay(int)));

  connect(d->UI.camera, SIGNAL(valueChanged(int)),
          this, SLOT(setActiveCamera(int)));

  this->setSlideDelay(d->UI.slideDelay->value());

  // Set up UI persistence and restore previous state
  d->uiState.mapState("Window/state", this);
  d->uiState.mapGeometry("Window/geometry", this);
  d->uiState.restore();
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
    "All Supported Files (*.conf *.ply *.krtd);;"
    "Project configuration file (*.conf);;"
    "Landmark file (*.ply);;"
    "Camera file (*.krtd);;"
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
  if (fi.suffix().toLower() == "conf")
  {
    this->loadProject(path);
  }
  else if (fi.suffix().toLower() == "ply")
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
void MainWindow::loadProject(const QString& path)
{
  QTE_D();

  Project project;
  if (!project.read(path))
  {
    qWarning() << "Failed to load project from" << path; // TODO dialog?
    return;
  }

  this->loadLandmarks(project.landmarks);

  foreach (auto const& ip, project.images)
  {
    QFileInfo fi(ip);
    QString cameraPath =
      project.cameraPath + "/" + fi.completeBaseName() + ".krtd";

    // Build camera data
    CameraData cd;
    cd.imagePath = ip;

    // Add camera to scene
    d->addCamera(cd, cameraPath);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadCamera(const QString& path)
{
  QTE_D();

  CameraData cd;
  d->addCamera(cd, path);
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(const QString& path)
{
  QTE_D();

  auto const& landmarks = maptk::read_ply_file(qPrintable(path));
  if (landmarks)
  {
    d->landmarks = landmarks;
    d->UI.worldView->addLandmarks(*landmarks);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setSlideDelay(int delayExp)
{
  QTE_D();

  auto const de = static_cast<double>(delayExp) * 0.1;
  d->slideTimer.setInterval(qRound(pow(10.0, de)));
}

//-----------------------------------------------------------------------------
void MainWindow::setSlideshowPlaying(bool playing)
{
  QTE_D();
  if (playing)
  {
    if (d->UI.camera->value() == d->UI.camera->maximum())
    {
      d->UI.camera->triggerAction(QAbstractSlider::SliderToMinimum);
    }
    d->slideTimer.start();
  }
  else
  {
    d->slideTimer.stop();
  }

  d->UI.camera->setEnabled(!playing);
}

//-----------------------------------------------------------------------------
void MainWindow::nextSlide()
{
  QTE_D();

  if (d->UI.camera->value() == d->UI.camera->maximum())
  {
    if (d->UI.actionSlideshowLoop->isChecked())
    {
      d->UI.camera->triggerAction(QAbstractSlider::SliderToMinimum);
    }
    else
    {
      d->UI.actionSlideshowPlay->setChecked(false);
    }
  }
  else
  {
    d->UI.camera->triggerAction(QAbstractSlider::SliderSingleStepAdd);
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setActiveCamera(int id)
{
  QTE_D();

  if (id < 0 || id >= d->cameras.count())
  {
    qDebug() << "MainWindow::setActiveCamera:"
             << " requested ID" << id << "is invalid";
    return;
  }

  // Show camera image
  auto const& cd = d->cameras[id];

  int imageDim[2];
  cd.camera->GetImageDimensions(imageDim);
  QSize imageDimensions(imageDim[0], imageDim[1]);
  d->UI.cameraView->loadImage(cd.imagePath, imageDimensions);
  cd.camera->SetImageDimensions(imageDimensions.width(),
                                imageDimensions.height());

  d->UI.cameraView->clearLandmarks();
  if (d->landmarks)
  {
    // Map landmarks to camera space
    double projectedPoint[2];
    auto const& landmarks = d->landmarks->landmarks();
    foreach_iter (auto, lmi, landmarks)
    {
      if (cd.camera->ProjectPoint(lmi->second->loc(), projectedPoint))
      {
        // Add projected landmark to camera view
        d->UI.cameraView->addLandmark(lmi->first,
          projectedPoint[0], projectedPoint[1]);
      }
    }
  }
}
