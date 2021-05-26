// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

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
