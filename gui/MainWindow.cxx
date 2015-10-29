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

#include "ui_MainWindow.h"
#include "am_MainWindow.h"

#include "AboutDialog.h"
#include "Project.h"
#include "Version.h"
#include "vtkMaptkCamera.h"

#include <maptk/camera_io.h>
#include <maptk/landmark_map_io.h>
#include <maptk/track_set_io.h>

#include <vtkSmartPointer.h>

#include <qtMath.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QApplication>
#include <QtGui/QDesktopServices>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>

#include <QtCore/QDebug>
#include <QtCore/QTimer>
#include <QtCore/QUrl>

namespace // anonymous
{

//-----------------------------------------------------------------------------
struct CameraData
{
  int id;
  vtkSmartPointer<vtkMaptkCamera> camera;

  QString imagePath; // Full path to camera image data
};

//-----------------------------------------------------------------------------
QString findUserManual()
{
  static auto const name = "gui.html";
  static auto const product = "maptk";
  static auto const version = MAPTK_VERSION;

  auto const& prefix =
    QFileInfo(QApplication::applicationFilePath()).dir().absoluteFilePath("..");

  auto locations = QStringList();

  // Install location
  locations.append(QString("%1/share/doc/%2-%3").arg(prefix, product, version));

  // Build location
  locations.append(QString("%1/doc").arg(prefix));

  foreach (auto const& path, locations)
  {
    auto const fi = QFileInfo(QString("%1/user/%2").arg(path, name));
    if (fi.exists())
    {
      // Found manual
      return fi.canonicalFilePath();
    }
  }

  // Manual not found
  return QString();
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class MainWindowPrivate
{
public:
  MainWindowPrivate() : activeCameraIndex(-1) {}

  void addCamera(maptk::camera_d const& camera,
                 QString const& imagePath = QString());

  void setActiveCamera(int);
  void updateCameraView();

  Ui::MainWindow UI;
  Am::MainWindow AM;
  qtUiState uiState;

  QTimer slideTimer;

  QList<CameraData> cameras;
  maptk::track_set_sptr tracks;
  maptk::landmark_map_sptr landmarks;

  int activeCameraIndex;
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
void MainWindowPrivate::addCamera(
  maptk::camera_d const& camera, QString const& imagePath)
{
  CameraData cd;

  cd.id = this->cameras.count();

  cd.imagePath = imagePath;

  cd.camera = vtkSmartPointer<vtkMaptkCamera>::New();
  cd.camera->SetCamera(camera);
  cd.camera->Update();

  this->cameras.append(cd);

  this->UI.worldView->addCamera(cd.id, cd.camera);

  this->UI.actionSlideshowPlay->setEnabled(true);
  this->UI.camera->setEnabled(true);
  this->UI.cameraSpin->setEnabled(true);
  this->UI.camera->setRange(0, this->cameras.count() - 1);
  this->UI.cameraSpin->setRange(0, this->cameras.count() - 1);

  // When the first camera is added, show it immediately and reset the camera
  // view
  if (this->cameras.count() == 1)
  {
    this->setActiveCamera(0);
    this->UI.cameraView->resetView();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::setActiveCamera(int id)
{
  this->activeCameraIndex = id;
  this->UI.worldView->setActiveCamera(this->cameras[id].camera);
  this->updateCameraView();
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::updateCameraView()
{
  if (this->activeCameraIndex < 0)
  {
    this->UI.cameraView->loadImage(QString(), 0);
    this->UI.cameraView->setActiveFrame(static_cast<unsigned>(-1));
    this->UI.cameraView->clearLandmarks();
    return;
  }

  this->UI.cameraView->setActiveFrame(static_cast<unsigned>(activeCameraIndex));

  QHash<maptk::track_id_t, maptk::vector_2d> landmarkPoints;

  auto const& cd = this->cameras[this->activeCameraIndex];

  // Show camera image
  this->UI.cameraView->loadImage(cd.imagePath, cd.camera);

  // Show landmarks
  this->UI.cameraView->clearLandmarks();
  if (this->landmarks)
  {
    // Map landmarks to camera space
    auto const& landmarks = this->landmarks->landmarks();
    foreach_iter (auto, lmi, landmarks)
    {
      double pp[2];
      if (cd.camera->ProjectPoint(lmi->second->loc(), pp))
      {
        // Add projected landmark to camera view
        auto const id = lmi->first;
        this->UI.cameraView->addLandmark(id, pp[0], pp[1]);
        landmarkPoints.insert(id, maptk::vector_2d(pp[0], pp[1]));
      }
    }
  }

  // Show residuals
  this->UI.cameraView->clearResiduals();
  if (this->tracks)
  {
    auto const& tracks = this->tracks->tracks();
    foreach (auto const& track, tracks)
    {
      auto const& state = track->find(this->activeCameraIndex);
      if (state != track->end() && state->feat)
      {
        auto const id = track->id();
        if (landmarkPoints.contains(id))
        {
          auto const& fp = state->feat->loc();
          auto const& lp = landmarkPoints[id];
          this->UI.cameraView->addResidual(id, fp[0], fp[1], lp[0], lp[1]);
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags), d_ptr(new MainWindowPrivate)
{
  QTE_D();

  // Set up UI
  d->UI.setupUi(this);
  d->AM.setupActions(d->UI, this);

  d->UI.menuView->addSeparator();
  d->UI.menuView->addAction(d->UI.cameraViewDock->toggleViewAction());
  d->UI.menuView->addAction(d->UI.cameraSelectorDock->toggleViewAction());

  d->UI.playSlideshowButton->setDefaultAction(d->UI.actionSlideshowPlay);
  d->UI.loopSlideshowButton->setDefaultAction(d->UI.actionSlideshowLoop);

  connect(d->UI.actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));
  connect(d->UI.actionQuit, SIGNAL(triggered()), qApp, SLOT(quit()));

  connect(d->UI.actionAbout, SIGNAL(triggered()),
          this, SLOT(showAboutDialog()));
  connect(d->UI.actionShowManual, SIGNAL(triggered()),
          this, SLOT(showUserManual()));

  connect(&d->slideTimer, SIGNAL(timeout()), this, SLOT(nextSlide()));
  connect(d->UI.actionSlideshowPlay, SIGNAL(toggled(bool)),
          this, SLOT(setSlideshowPlaying(bool)));
  connect(d->UI.slideDelay, SIGNAL(valueChanged(int)),
          this, SLOT(setSlideDelay(int)));

  connect(d->UI.camera, SIGNAL(valueChanged(int)),
          this, SLOT(setActiveCamera(int)));

  this->setSlideDelay(d->UI.slideDelay->value());

  // Set up UI persistence and restore previous state
  auto const sdItem = new qtUiState::Item<int, QSlider>(
    d->UI.slideDelay, &QSlider::value, &QSlider::setValue);
  d->uiState.map("SlideDelay", sdItem);

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
    "All Supported Files (*.conf *.txt *.ply *.krtd);;"
    "Project configuration file (*.conf);;"
    "Track file (*.txt);;"
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
  else if (fi.suffix().toLower() == "txt")
  {
    this->loadTracks(path);
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

  this->loadTracks(project.tracks);
  this->loadLandmarks(project.landmarks);

  auto const cameraDir = maptk::path_t(qPrintable(project.cameraPath));
  foreach (auto const& ip, project.images)
  {
    try
    {
      auto const& camera = maptk::read_krtd_file(qPrintable(ip), cameraDir);

      // Add camera to scene
      d->addCamera(camera, ip);
    }
    catch (...)
    {
      qWarning() << "failed to read camera for" << ip
                 << "from" << project.cameraPath;
    }
  }

  d->UI.worldView->resetView();
}

//-----------------------------------------------------------------------------
void MainWindow::loadCamera(const QString& path)
{
  QTE_D();

  try
  {
    auto const& camera = maptk::read_krtd_file(qPrintable(path));
    d->addCamera(camera);
  }
  catch (...)
  {
    qWarning() << "failed to read camera from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadTracks(QString const& path)
{
  QTE_D();

  try
  {
    auto const& tracks = maptk::read_track_file(qPrintable(path));
    if (tracks)
    {
      d->tracks = tracks;
      d->updateCameraView();

      foreach (auto const& track, tracks->tracks())
      {
        d->UI.cameraView->addFeatureTrack(*track);
      }
    }
  }
  catch (...)
  {
    qWarning() << "failed to read tracks from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(const QString& path)
{
  QTE_D();

  try
  {
    auto const& landmarks = maptk::read_ply_file(qPrintable(path));
    if (landmarks)
    {
      d->landmarks = landmarks;
      d->UI.worldView->addLandmarks(*landmarks);
    }
  }
  catch (...)
  {
    qWarning() << "failed to read landmarks from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::setSlideDelay(int delayExp)
{
  QTE_D();

  static auto const ttFormat =
    QString("%1 (%2)").arg(d->UI.slideDelay->toolTip());

  auto const de = static_cast<double>(delayExp) * 0.1;
  auto const delay = qRound(qPow(10.0, de));
  d->slideTimer.setInterval(delay);

  if (delay < 1000)
  {
    auto const fps = 1e3 / delay;
    auto const dt = QString("%1 / sec").arg(fps, 0, 'f', 1);
    d->UI.slideDelay->setToolTip(ttFormat.arg(dt));
  }
  else
  {
    auto const dt = QString("%1 sec").arg(delay / 1e3, 0, 'f', 1);
    d->UI.slideDelay->setToolTip(ttFormat.arg(dt));
  }
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

  d->setActiveCamera(id);
}

//-----------------------------------------------------------------------------
void MainWindow::showAboutDialog()
{
  AboutDialog dlg(this);
  dlg.exec();
}

//-----------------------------------------------------------------------------
void MainWindow::showUserManual()
{
  auto const path = findUserManual();
  if (!path.isEmpty())
  {
    auto const& uri = QUrl::fromLocalFile(path);
    QDesktopServices::openUrl(uri);
  }
  else
  {
    QMessageBox::information(
      this, "Not Found",
      "The user manual could not be located. Please check your installation.");
  }
}
