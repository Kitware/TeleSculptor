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

#include <QApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <iostream>

namespace
{
static QString const CAMERA_PATH = "results/krtd";
static QString const TRACKS_PATH = "results/tracks.txt";
static QString const LANDMARKS_PATH = "results/landmarks.ply";
static QString const GEO_ORIGIN_PATH = "results/geo_origin.txt";
static QString const DEPTH_PATH = "results/depth";
}

//-----------------------------------------------------------------------------
QString getPath(kwiver::vital::config_block_sptr const& config,
                QDir const& base, std::string const& key,
                QString const& defaultPath = {},
                std::string const& altKey = {})
{
  try
  {
    auto const& value = config->get_value<std::string>(key);
    return base.filePath(qtString(value));
  }
  catch (...)
  {
    return (!altKey.empty() ? getPath(config, base, altKey, defaultPath) :
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
    auto const& config = kwiver::vital::read_config_file(
      qPrintable(path), "maptk", MAPTK_VERSION, prefix);

    if (config->has_value("working_directory"))
    {
      this->workingDir = qtString(
        config->get_value<std::string>("working_directory"));
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
    this->videoPath =
      getPath(config, this->workingDir, "video_source", {}, "image_list_file");
    this->maskPath =
      getPath(config, this->workingDir, "mask_source");

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
  auto rel = workingDir.relativeFilePath(filepath);
  if (rel.startsWith(".."))
  {
    return filepath;
  }
  else
  {
    return rel;
  }
}

//-----------------------------------------------------------------------------
void Project::write()
{
  if (!this->videoPath.isEmpty())
  {
    this->projectConfig->set_value("video_source",
      stdString(this->getContingentRelativePath(this->videoPath)));
  }
  if (!this->maskPath.isEmpty())
  {
    this->projectConfig->set_value("mask_source",
      stdString(this->getContingentRelativePath(this->maskPath)));
  }

  if (this->projectConfig->available_values().size() > 0)
  {
    kwiver::vital::write_config_file(
      this->projectConfig, stdString(this->filePath));
  }
}
