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

#ifndef MAPTK_WORLDVIEW_H_
#define MAPTK_WORLDVIEW_H_

#include <qtGlobal.h>

#include <QtGui/QWidget>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

#include "DepthMapPaths.h"

class vtkImageData;

class vtkPolyData;

namespace kwiver { namespace vital { class landmark_map; } }

class vtkMaptkCamera;

class WorldViewPrivate;

class WorldView : public QWidget
{
  Q_OBJECT

public:
  explicit WorldView(QWidget* parent = 0, Qt::WindowFlags flags = 0);
  virtual ~WorldView();

//  void addDepthMaps(QString const& dMFileList, std::string type);

  std::string getActiveDepthMapType();

  void setActiveDepthMap(vtkMaptkCamera *cam, QString vtiPath);
  void enableDepthMap();

  void loadVolume(QString path, int nbFrames, QString krtd, QString frame);


signals:
  void contourChanged();
  void activeDepthMapChanged(int);
  void meshEnabled(bool);
  void coloredMeshEnabled(bool);
  void updateThresholds(double,double,double,double);

public slots:
  void setBackgroundColor(QColor const&);

  void addCamera(int id, vtkMaptkCamera* camera);
  void setLandmarks(kwiver::vital::landmark_map const&);

  void setImageData(vtkImageData* data, QSize const& dimensions);

  void setImageVisible(bool);
  void setCamerasVisible(bool);
  void setLandmarksVisible(bool);
  void setGroundPlaneVisible(bool);
  void setDepthMapVisible(bool);

  void setGlobalGridVisible(bool state);

  void updateGrid();

  void setVolumeVisible(bool);
  void setVolumeCurrentFramePath(QString path);

  void setPerspective(bool);

  void setActiveCamera(vtkMaptkCamera* camera);

  void resetView();
  void resetViewToLandmarks();

  void viewToWorldTop();
  void viewToWorldLeft();
  void viewToWorldRight();
  void viewToWorldFront();
  void viewToWorldBack();

  void saveMesh(QString const& path);
  void saveVolume(QString const& path);
  void saveColoredMesh(QString const& path);

  void updateDepthMapRepresentation();
  void updateDepthMapThresholds();

  void computeContour(double threshold);


  void exportWebGLScene(QString const& path);
protected slots:
  void updateCameras();
  void updateScale();

public slots:


private:

  QTE_DECLARE_PRIVATE_RPTR(WorldView)
  QTE_DECLARE_PRIVATE(WorldView)

  QTE_DISABLE_COPY(WorldView)
};

#endif
