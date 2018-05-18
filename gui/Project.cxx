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

#include <kwiversys/SystemTools.hxx>

#include <qtStlUtil.h>

#include <QApplication>

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <iostream>

namespace
{
static char const* const CAMERA_PATH = "results/krtd";
static char const* const TRACKS_PATH = "results/tracks.txt";
static char const* const LANDMARKS_PATH = "results/landmarks.ply";
static char const* const GEO_ORIGIN_PATH = "results/geo_origin.txt";
static char const* const DEPTH_PATH = "results/depth";
static std::string WORKING_DIR_TAG = "working_directory";
static std::string VIDEO_SOURCE_TAG = "video_source";
static char const* const IMAGE_LIST_FILE = "image_list_file";
}

//-----------------------------------------------------------------------------
QString getPath(kwiver::vital::config_block_sptr const& config,
                QDir const& base, char const* key,
                char const* defaultPath = 0, char const* altKey = 0)
{
  try
  {
    auto const& value = config->get_value<std::string>(key);
    return base.filePath(qtString(value));
  }
  catch (...)
  {
    return (altKey ? getPath(config, base, altKey, defaultPath) :
            QString(defaultPath));
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
  filePath = workingDir.absoluteFilePath(workingDir.dirName() + ".conf");

  // Set default paths
  cameraPath = CAMERA_PATH;
  tracksPath = TRACKS_PATH;
  landmarksPath = LANDMARKS_PATH;
  geoOriginFile = GEO_ORIGIN_PATH;
  depthPath = DEPTH_PATH;
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

    filePath = path;

    this->cameraPath =
      getPath(config, this->workingDir, "output_krtd_dir", CAMERA_PATH);
    this->landmarksPath =
      getPath(config, this->workingDir, "output_ply_file", LANDMARKS_PATH);
    this->tracksPath =
      getPath(config, this->workingDir, "input_track_file",
              TRACKS_PATH, "output_tracks_file");
    this->depthPath =
      getPath(config, this->workingDir, "output_depth_dir", DEPTH_PATH);


    // Read Volume file
    if (config->has_value("volume_file"))
    {
      this->volumePath = getPath(config, this->workingDir, "volume_file");
    }

    // Read the geo origin file
    if (config->has_value("geo_origin_file"))
    {
      this->geoOriginFile =
        getPath(config, this->workingDir, "geo_origin_file", GEO_ORIGIN_PATH);
    }

    // Read video file
    if (config->has_value(VIDEO_SOURCE_TAG) ||
        config->has_value(IMAGE_LIST_FILE))
    {
      this->videoPath = getPath(config, this->workingDir,
                                VIDEO_SOURCE_TAG.c_str(),
                                "", IMAGE_LIST_FILE);
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

//-----------------------------------------------------------------------------
// Returns the relative path if the filepath is contained in the directory and
// returns the absolute path if not.
QString Project::getContingentRelativePath(QString filepath)
{
  if (kwiversys::SystemTools::IsSubDirectory(filepath.toStdString(),
                                             workingDir.absolutePath().toStdString()))
  {
    return workingDir.relativeFilePath(filepath);
  }
  else
  {
    return filepath;
  }
}

void Project::write()
{
  if (!videoPath.isEmpty())
  {
    projectConfig->set_value(VIDEO_SOURCE_TAG,
      getContingentRelativePath(videoPath).toStdString());
  }

  if (projectConfig->available_values().size() > 0)
  {
    kwiver::vital::write_config_file(projectConfig, filePath.toStdString());
  }
}
