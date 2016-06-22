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

#ifndef MAPTK_MAINWINDOW_H_
#define MAPTK_MAINWINDOW_H_

#include <qtGlobal.h>

#include <QMainWindow>

class MainWindowPrivate;

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

  void loadProject(QString const& path);
  void loadImage(QString const& path);
  void loadCamera(QString const& path);
  void loadTracks(QString const& path);
  void loadLandmarks(QString const& path);

  void saveCameras();
  void saveCameras(QString const& path);
  void saveLandmarks();
  void saveLandmarks(QString const& path);

  void setActiveCamera(int);

  void setViewBackroundColor();

  void showMatchMatrix();

  void showAboutDialog();
  void showUserManual();

  void exportWebGLScene();

protected slots:
  void setSlideDelay(int);
  void setSlideshowPlaying(bool);
  void nextSlide();

  void executeTool(QObject*);
  void acceptToolResults();

private:
  QTE_DECLARE_PRIVATE_RPTR(MainWindow)
  QTE_DECLARE_PRIVATE(MainWindow)

  QTE_DISABLE_COPY(MainWindow)
};

#endif
