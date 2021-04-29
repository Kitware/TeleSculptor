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

#ifndef TELESCULPTOR_PROJECT_H_
#define TELESCULPTOR_PROJECT_H_

#include <vital/config/config_block_io.h>

#include <QtCore/QDir>
#include <QtCore/QMap>
#include <QtCore/QStringList>

class Project : public QObject
{
  Q_OBJECT

// TODO: Encapsulate data and add accessors
public:

  Project(QObject* parent = nullptr);
  Project(QString const& dir, QObject* parent = nullptr);

  bool read(QString const& path);

  QString getContingentRelativePath(QString const& filepath);

  QString logFileName() const;

  QDir workingDir;

  QString filePath;
  QString videoPath;
  QString maskPath;

  QString tracksPath;
  QString landmarksPath;
  QString volumePath;
  QString cameraPath;
  QString geoOriginFile;
  QString depthPath;
  QString meshPath;
  QString groundControlPath;
  QString logFilePath;

  std::string ROI;

  kwiver::vital::config_block_sptr config =
    kwiver::vital::config_block::empty_config();

public slots:
  void write();
};

#endif
