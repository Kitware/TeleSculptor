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

#ifndef MAPTK_PROJECT_H_
#define MAPTK_PROJECT_H_

#include <vital/config/config_block_io.h>
#include <kwiversys/SystemTools.hxx>

#include <QtCore/QDir>
#include <QtCore/QMap>
#include <QtCore/QStringList>

class Project : public QObject
{
  Q_OBJECT

// TODO: Encapsulate data and add accessors
public:

  Project();
  Project(QString dir);

  bool read(QString const& path);

  QString getContingentRelativePath(QString filepath);

  QDir workingDir;

  QString filePath;
  QString videoPath;

  QString tracksPath;
  QString landmarksPath;
  QString volumePath;
  QString cameraPath;
  QString geoOriginFile;
  QString depthPath;

  kwiver::vital::config_block_sptr projectConfig;

public slots:
  void write();

  //----------------------------------------------------------------------------
  // find the full path to the first matching file on the config search path
  static kwiver::vital::path_t findConfig(std::string const& name)
  {
    try
    {
      using kwiver::vital::application_config_file_paths;

      auto const exeDir = QDir(QApplication::applicationDirPath());
      auto const prefix = stdString(exeDir.absoluteFilePath(".."));
      auto const& search_paths =
        application_config_file_paths("maptk", MAPTK_VERSION, prefix);

      for (auto const& search_path : search_paths)
      {
        auto const& config_path = search_path + "/" + name;

        if (kwiversys::SystemTools::FileExists(config_path) &&
          !kwiversys::SystemTools::FileIsDirectory(config_path))
        {
          return config_path;
        }
      }
    }
    catch (...)
    {
      return "";
    }
    return "";
  }

};

#endif
