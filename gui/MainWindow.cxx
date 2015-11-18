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
#include "MatchMatrixWindow.h"
#include "Project.h"
#include "Version.h"
#include "vtkMaptkCamera.h"

#include <maptk/match_matrix.h>

#include <vital/io/camera_io.h>
#include <vital/io/landmark_map_io.h>
#include <vital/io/track_set_io.h>

#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Collection.h>
#include <vtkImageReader2Factory.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>

#include <qtMath.h>
#include <qtUiState.h>
#include <qtUiStateItem.h>

#include <QtGui/QApplication>
#include <QtGui/QDesktopServices>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>

#include <QtCore/QDebug>
#include <QtCore/QQueue>
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

//-----------------------------------------------------------------------------
QSet<QString> supportedImageExtensions()
{
  QSet<QString> result;

  auto const whitespace = QRegExp("\\s");

  // Get registered readers
  vtkNew<vtkImageReader2Collection> readers;
  vtkImageReader2Factory::GetRegisteredReaders(readers.GetPointer());

  // Extract extensions for each reader
  readers->InitTraversal();
  while (auto const reader = readers->GetNextItem())
  {
    auto const extensionList =
      QString::fromLocal8Bit(reader->GetFileExtensions());
    auto const& extensions =
      extensionList.split(whitespace, QString::SkipEmptyParts);

    foreach (auto const& ext, extensions)
    {
      result.insert(ext.mid(1).toLower());
    }
  }

  return result;
}

//-----------------------------------------------------------------------------
QString makeFilters(QStringList extensions)
{
  auto result = QStringList();
  foreach (auto const& extension, extensions)
  {
    result.append("*." + extension);
  }
  return result.join(" ");
}

} // namespace <anonymous>

//-----------------------------------------------------------------------------
class MainWindowPrivate
{
public:
  MainWindowPrivate() : activeCameraIndex(-1) {}

  void addCamera(kwiver::vital::camera_sptr const& camera);
  void addImage(QString const& imagePath);

  void addFrame(kwiver::vital::camera_sptr const& camera,
                QString const& imagePath);

  void setActiveCamera(int);
  void updateCameraView();

  void loadImage(QString const& path, vtkMaptkCamera* camera);

  Ui::MainWindow UI;
  Am::MainWindow AM;
  qtUiState uiState;

  QTimer slideTimer;

  QList<CameraData> cameras;
  kwiver::vital::track_set_sptr tracks;
  kwiver::vital::landmark_map_sptr landmarks;

  int activeCameraIndex;

  QQueue<int> orphanImages;
  QQueue<int> orphanCameras;
};

QTE_IMPLEMENT_D_FUNC(MainWindow)

//-----------------------------------------------------------------------------
void MainWindowPrivate::addCamera(kwiver::vital::camera_sptr const& camera)
{
  if (this->orphanImages.isEmpty())
  {
    this->orphanCameras.enqueue(this->cameras.count());
    this->addFrame(camera, QString());
    return;
  }

  auto& cd = this->cameras[this->orphanImages.dequeue()];

  cd.camera = vtkSmartPointer<vtkMaptkCamera>::New();
  cd.camera->SetCamera(camera);
  cd.camera->Update();

  this->UI.worldView->addCamera(cd.id, cd.camera);
  if (cd.id == this->activeCameraIndex)
  {
    this->UI.worldView->setActiveCamera(cd.camera);
    this->updateCameraView();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addImage(QString const& imagePath)
{
  if (this->orphanCameras.isEmpty())
  {
    this->orphanImages.enqueue(this->cameras.count());
    this->addFrame(0, imagePath);
    return;
  }

  auto& cd = this->cameras[this->orphanCameras.dequeue()];

  cd.imagePath = imagePath;

  if (cd.id == this->activeCameraIndex)
  {
    this->updateCameraView();
  }
}

//-----------------------------------------------------------------------------
void MainWindowPrivate::addFrame(
  kwiver::vital::camera_sptr const& camera, QString const& imagePath)
{
  CameraData cd;

  cd.id = this->cameras.count();

  cd.imagePath = imagePath;

  if (camera)
  {
    this->orphanImages.clear();

    cd.camera = vtkSmartPointer<vtkMaptkCamera>::New();
    cd.camera->SetCamera(camera);
    cd.camera->Update();

    this->UI.worldView->addCamera(cd.id, cd.camera);
  }
  else
  {
    this->orphanCameras.clear();
  }

  this->cameras.append(cd);

  this->UI.camera->setRange(0, this->cameras.count() - 1);
  this->UI.cameraSpin->setRange(0, this->cameras.count() - 1);

  // When the first camera is added, show it immediately and reset the camera
  // view, and enable slideshow controls
  if (this->cameras.count() == 1)
  {
    this->UI.actionSlideshowPlay->setEnabled(true);
    this->UI.camera->setEnabled(true);
    this->UI.cameraSpin->setEnabled(true);

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
    this->loadImage(QString(), 0);
    this->UI.cameraView->setActiveFrame(static_cast<unsigned>(-1));
    this->UI.cameraView->clearLandmarks();
    return;
  }

  this->UI.cameraView->setActiveFrame(
    static_cast<unsigned>(this->activeCameraIndex));

  QHash<kwiver::vital::track_id_t, kwiver::vital::vector_2d> landmarkPoints;

  auto const& cd = this->cameras[this->activeCameraIndex];

  // Show camera image
  this->loadImage(cd.imagePath, cd.camera);

  if (!cd.camera)
  {
    // Can't show landmarks or residuals with no camera
    this->UI.cameraView->clearLandmarks();
    this->UI.cameraView->clearResiduals();
    return;
  }

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
        landmarkPoints.insert(id, kwiver::vital::vector_2d(pp[0], pp[1]));
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
void MainWindowPrivate::loadImage(QString const& path, vtkMaptkCamera* camera)
{
  if (path.isEmpty())
  {
    auto imageDimensions = QSize(1, 1);
    if (camera)
    {
      int w, h;
      camera->GetImageDimensions(w, h);
      imageDimensions = QSize(w, h);
    }

    this->UI.cameraView->setImageData(0, imageDimensions);
    this->UI.worldView->setImageData(0, imageDimensions);
  }
  else
  {
    // Create a reader capable of reading the image file
    auto const reader =
      vtkImageReader2Factory::CreateImageReader2(qPrintable(path));
    if (!reader)
    {
      qWarning() << "Failed to create image reader for image" << path;
      this->loadImage(QString(), camera);
      return;
    }

    // Load the image
    reader->SetFileName(qPrintable(path));
    reader->Update();

    // Get dimensions
    auto const data = reader->GetOutput();
    int dimensions[3];
    data->GetDimensions(dimensions);

    // Test for errors
    if (dimensions[0] < 2 || dimensions[1] < 2)
    {
      qWarning() << "Failed to read image" << path;
      this->loadImage(QString(), camera);
    }
    else
    {
      // If successful, update camera image dimensions
      if (camera)
      {
        camera->SetImageDimensions(dimensions);
      }

      // Set image on views
      auto const size = QSize(dimensions[0], dimensions[1]);
      this->UI.cameraView->setImageData(data, size);
      this->UI.worldView->setImageData(data, size);
    }

    // Delete the reader
    reader->Delete();
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

  connect(d->UI.actionShowMatchMatrix, SIGNAL(triggered()),
          this, SLOT(showMatchMatrix()));

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
  static auto const imageFilters =
    makeFilters(supportedImageExtensions().toList());

  auto const paths = QFileDialog::getOpenFileNames(
    this, "Open File", QString(),
    "All Supported Files (*.conf *.txt *.ply *.krtd " + imageFilters + ");;"
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
  static auto const imageExtensions = supportedImageExtensions();

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
  else if (imageExtensions.contains(fi.suffix().toLower()))
  {
    this->loadImage(path);
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
void MainWindow::loadProject(QString const& path)
{
  QTE_D();

  Project project;
  if (!project.read(path))
  {
    qWarning() << "Failed to load project from" << path; // TODO dialog?
    return;
  }

  if (!project.tracks.isEmpty())
  {
    this->loadTracks(project.tracks);
  }
  if (!project.landmarks.isEmpty())
  {
    this->loadLandmarks(project.landmarks);
  }

  if (project.cameraPath.isEmpty())
  {
    foreach (auto const& ip, project.images)
    {
      d->addImage(ip);
    }
  }
  else
  {
    auto const cameraDir = kwiver::vital::path_t(qPrintable(project.cameraPath));
    foreach (auto const& ip, project.images)
    {
      try
      {
        auto const& camera =
          kwiver::vital::read_krtd_file(qPrintable(ip), cameraDir);

        // Add camera to scene
        d->addFrame(camera, ip);
      }
      catch (...)
      {
        qWarning() << "failed to read camera for" << ip
                   << "from" << project.cameraPath;
        d->addFrame(0, ip);
      }
    }
  }

  d->UI.worldView->resetView();
}

//-----------------------------------------------------------------------------
void MainWindow::loadImage(QString const& path)
{
  QTE_D();
  d->addImage(path);
}

//-----------------------------------------------------------------------------
void MainWindow::loadCamera(QString const& path)
{
  QTE_D();

  try
  {
    auto const& camera = kwiver::vital::read_krtd_file(qPrintable(path));
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
    auto const& tracks = kwiver::vital::read_track_file(qPrintable(path));
    if (tracks)
    {
      d->tracks = tracks;
      d->updateCameraView();

      foreach (auto const& track, tracks->tracks())
      {
        d->UI.cameraView->addFeatureTrack(*track);
      }

      d->UI.actionShowMatchMatrix->setEnabled(!tracks->tracks().empty());
    }
  }
  catch (...)
  {
    qWarning() << "failed to read tracks from" << path;
  }
}

//-----------------------------------------------------------------------------
void MainWindow::loadLandmarks(QString const& path)
{
  QTE_D();

  try
  {
    auto const& landmarks = kwiver::vital::read_ply_file(qPrintable(path));
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
void MainWindow::showMatchMatrix()
{
  QTE_D();

  if (d->tracks)
  {
    // Get matrix
    auto frames = std::vector<kwiver::vital::frame_id_t>();
    auto const mm = kwiver::maptk::match_matrix(d->tracks, frames);

    // Show window
    auto window = new MatchMatrixWindow();
    window->setMatrix(mm);
    window->show();
  }
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
