/*ckwg +29
 * Copyright 2016-2017 by Kitware, Inc.
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

#ifndef MAPTK_MAINWINDOW_H_
#define MAPTK_MAINWINDOW_H_

#include <qtGlobal.h>

#include <QMainWindow>

#include <memory>

class ToolData;

class MainWindowPrivate;

class VideoData;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~MainWindow();

public slots:
  void openFile();
  void openFile(QString const& path);
  void openFiles(QStringList const& paths);

  void newProject();

  void loadProject(QString const& path);
  void loadImage(QString const& path);
  void loadVideo(QString const& path);
  void loadCamera(QString const& path);
  void loadTracks(QString const& path);
  void loadLandmarks(QString const& path);

  void saveCameras();
  void saveCameras(QString const& path, bool writeToProject = true);
  void saveLandmarks();
  void saveLandmarks(QString const& path, bool writeToProject = true);
  void saveTracks();
  void saveTracks(QString const& path, bool writeToProject = true);
  void saveDepthPoints();
  void saveDepthPoints(QString const& path);
  void saveDepthImage(QString const& path);
  void saveGeoOrigin(QString const& path);
  void saveToolResults();


  void saveWebGLScene();

  void enableSaveMesh(bool);
  void enableSaveColoredMesh(bool);
  void saveMesh();
  void saveVolume();
  void saveColoredMesh();

  void enableSaveDepthPoints(bool);

  void setActiveCamera(int);

  void setViewBackroundColor();

  void showMatchMatrix();

  void showAboutDialog();
  void showUserManual();

protected slots:
  void setSlideDelay(int);
  void setSlideshowPlaying(bool);
  void nextSlide();

  void executeTool(QObject*);
  void acceptToolFinalResults();
  void acceptToolResults(std::shared_ptr<ToolData> data, bool isFinal = false);
  void updateToolResults();
  void addFrame(int);
  void updateFrames(std::shared_ptr<VideoData>);

private:
  QTE_DECLARE_PRIVATE_RPTR(MainWindow)
  QTE_DECLARE_PRIVATE(MainWindow)

  QTE_DISABLE_COPY(MainWindow)
};

#endif
