/*ckwg +29
 * Copyright 2016-2018 by Kitware, Inc.
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

#ifndef TELESCULPTOR_VOLUMEOPTIONS_H_
#define TELESCULPTOR_VOLUMEOPTIONS_H_

#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>

#include <qtGlobal.h>

#include <QWidget>

class vtkPolyData;

class vtkActor;

class VolumeOptionsPrivate;

class VolumeOptions : public QWidget
{
  Q_OBJECT

public:
  explicit VolumeOptions(QString const& settingsGroup,
      QWidget* parent = 0, Qt::WindowFlags flags = 0);
  ~VolumeOptions() override;

  void setActor(vtkActor* actor);

  void initFrameSampling(int nbFrames);
  int getFrameSampling() const;
  double getOcclusionThreshold() const;

  void setCameras(kwiver::vital::camera_map_sptr cameras);
  kwiver::vital::camera_map_sptr getCameras() const;
  void setVideoConfig(std::string const& videoPath,
                      kwiver::vital::config_block_sptr config);
  kwiver::vital::config_block_sptr getVideoConfig() const;
  std::string getVideoPath() const;
  void colorize();

  void setCurrentFrame(int);

  bool isColorOptionsEnabled();

signals:
  void currentFrameIDChanged(int);
  void modified();
  void colorOptionsEnabled(bool);

public slots:
  void showColorizeSurfaceMenu(bool state);
  void updateColorizeSurfaceMenu(QString const& text);

private:
  QTE_DECLARE_PRIVATE_RPTR(VolumeOptions)
  QTE_DECLARE_PRIVATE(VolumeOptions)

  QTE_DISABLE_COPY(VolumeOptions)
};

#endif
