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

#include "Project.h"

#include <maptk/version.h>

#include <qtStlUtil.h>

#include <QtGui/QApplication>

#include <QtCore/QDebug>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>

#include <iostream>

//-----------------------------------------------------------------------------
QString getPath(kwiver::vital::config_block_sptr const& config,
                QDir const& base, char const* key, char const* altKey = 0)
{
  try
  {
    auto const& value = config->get_value<std::string>(key);
    return base.filePath(qtString(value));
  }
  catch (...)
  {
    return (altKey ? getPath(config, base, altKey) : QString());
  }
}

//-----------------------------------------------------------------------------
// Returns the relative path if the filepath is contained in the directory and
// returns the absolute path if not.
QString getContingentRelativePath(QDir dir, QString filepath)
{
  if (filepath.startsWith(dir.absolutePath()))
  {
    return dir.relativeFilePath(filepath);
  }
  else
  {
    return filepath;
  }
}

//-----------------------------------------------------------------------------
Project::Project()
{
  projectConfig = kwiver::vital::config_block::empty_config();
}

//-----------------------------------------------------------------------------
Project::Project(QString dir)
{
  projectConfig = kwiver::vital::config_block::empty_config();

  workingDir = dir;
}

//-----------------------------------------------------------------------------
bool Project::read(QString const& path)
{
  auto const& base = QFileInfo(path).absoluteDir();

  try
  {
    // Load config file
    auto const exeDir = QDir(QApplication::applicationDirPath());
    auto const prefix = stdString(exeDir.absoluteFilePath(".."));
    auto const& config = kwiver::vital::read_config_file(qPrintable(path),
                                                         "maptk",
                                                         MAPTK_VERSION,
                                                         prefix);

    if (config->has_value(WORKING_DIR_TAG))
    {
      this->workingDir =
        QString::fromStdString(config->get_value<std::string>(WORKING_DIR_TAG));
    }
    else
    {
      this->workingDir = base;
    }

    this->cameraPath = getPath(config, base, "output_krtd_dir");
    this->landmarks = getPath(config, base, "output_ply_file");
    this->tracks =
      getPath(config, base, "input_track_file", "output_tracks_file");

    // Read depth map images list
    if (config->has_value("depthmaps_images_file"))
    {
      auto const& dmifPath =
        config->get_value<std::string>("depthmaps_images_file");
      auto const& dmifAbsolutePath = base.filePath(qtString(dmifPath));
      auto const& dmifBase = QFileInfo(dmifAbsolutePath).absoluteDir();
      QFile dmif(dmifAbsolutePath);
      if (!dmif.open(QIODevice::ReadOnly | QIODevice::Text))
      {
        // TODO set error
        return false;
      }

      auto parts = QRegExp("(\\d+)\\s+([^\n]+)\n*");
      while (!dmif.atEnd())
      {
        auto const& line = QString::fromLocal8Bit(dmif.readLine());
        if (parts.exactMatch(line))
        {
          this->depthMaps.insert(parts.cap(1).toInt(),
                                 dmifBase.filePath(parts.cap(2)));
        }
      }
    }

    //Read Volume file
    if (config->has_value("volume_file"))
    {
      this->volumePath = getPath(config, base, "volume_file");
    }

    // Read video file
    if (config->has_value(VIDEO_SOURCE_TAG) ||
        config->has_value("image_list_file"))
    {
      this->videoPath = getPath(config, base,
                                VIDEO_SOURCE_TAG.c_str(),
                                "image_list_file");
    }

    projectConfig = config;

    return true;
  }
  catch (kwiver::vital::config_block_exception e)
  {
    // TODO set error
    qWarning() << e.what(); // TODO dialog?
    return false;
  }
  catch (...)
  {
    // TODO set error
    return false;
  }
}

void Project::write()
{
  if (!videoPath.isEmpty())
  {
    projectConfig->set_value(VIDEO_SOURCE_TAG,
      getContingentRelativePath(workingDir, videoPath).toStdString());
  }

  if (projectConfig->available_values().size() > 0)
  {
    auto filePath = workingDir.absoluteFilePath(workingDir.dirName() + ".conf");
    kwiver::vital::write_config_file(projectConfig, filePath.toStdString());
  }
}