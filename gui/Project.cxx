// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "Project.h"

#include <maptk/version.h>

#include <qtStlUtil.h>

#include <QApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
static QString const CAMERA_PATH = "results/krtd";
static QString const TRACKS_PATH = "results/tracks.txt";
static QString const LANDMARKS_PATH = "results/landmarks.ply";
static QString const GEO_ORIGIN_PATH = "results/geo_origin.txt";
static QString const DEPTH_PATH = "results/depth";
static QString const GROUND_CONTROL_PATH = "results/ground_control_points.ply";
static QString const LOG_FILE_FMT = "telesculptor_%Y%m%d_%H%M%S.log";
}

//-----------------------------------------------------------------------------
QString getPath(Project const* project, std::string const& key,
                QString const& defaultPath = {},
                std::string const& altKey = {})
{
  try
  {
    auto const& value = project->config->get_value<std::string>(key);
    return project->workingDir.filePath(qtString(value));
  }
  catch (...)
  {
    return (!altKey.empty() ? getPath(project, altKey, defaultPath) : QString(defaultPath));
  }
}

//-----------------------------------------------------------------------------
Project::Project(QObject* parent) : QObject{parent}
{
}

//-----------------------------------------------------------------------------
Project::Project(QString const& dir, QObject* parent) : QObject{parent}
{
  workingDir = dir;
  filePath = workingDir.absoluteFilePath(workingDir.dirName() + ".conf");

  // Set default paths
  cameraPath = CAMERA_PATH;
  tracksPath = TRACKS_PATH;
  landmarksPath = LANDMARKS_PATH;
  geoOriginFile = GEO_ORIGIN_PATH;
  depthPath = DEPTH_PATH;
  groundControlPath = GROUND_CONTROL_PATH;
  logFilePath = this->logFileName();
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
    this->config = kwiver::vital::read_config_file(
      qPrintable(path), "telesculptor", TELESCULPTOR_VERSION, prefix);

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

    this->cameraPath = getPath(this, "output_krtd_dir", CAMERA_PATH);
    this->depthPath = getPath(this, "output_depth_dir", DEPTH_PATH);
    this->landmarksPath = getPath(this, "output_ply_file", LANDMARKS_PATH);
    this->tracksPath = getPath(this, "input_track_file", TRACKS_PATH,
                               "output_tracks_file");
    this->logFilePath = this->logFileName();

    if (config->has_value("ground_control_points_file"))
    {
      this->groundControlPath = getPath(this,
                                        "ground_control_points_file",
                                        GROUND_CONTROL_PATH);
    }

    // Read Volume file
    if (this->config->has_value("volume_file"))
    {
      this->volumePath = getPath(this, "volume_file");
    }

    // Read Mesh file
    if (this->config->has_value("mesh_file"))
    {
      this->meshPath = getPath(this, "mesh_file");
    }

    // Read the geo origin file
    if (this->config->has_value("geo_origin_file"))
    {
      this->geoOriginFile = getPath(this, "geo_origin_file", GEO_ORIGIN_PATH);
    }

    // Read video file
    this->videoPath = getPath(this, "video_source", {}, "image_list_file");
    this->maskPath = getPath(this, "mask_source");

    if (this->config->has_value("ROI"))
    {
      this->ROI = this->config->get_value<std::string>("ROI");
    }
    else
    {
      this->ROI = std::string("");
    }

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
QString Project::getContingentRelativePath(QString const& filepath)
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
// Construct the log filename from the format string and current date/time
QString Project::logFileName() const
{
  QString fmt = getPath(this, "log_filename_format", LOG_FILE_FMT);
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  std::stringstream ss;
  ss << std::put_time(&tm, qPrintable(fmt));
  return qtString(ss.str());
}

//-----------------------------------------------------------------------------
void Project::write()
{
  if (!this->videoPath.isEmpty())
  {
    this->config->set_value("video_source",
                            stdString(this->getContingentRelativePath(this->videoPath)));
  }
  if (!this->maskPath.isEmpty())
  {
    this->config->set_value("mask_source",
                            stdString(this->getContingentRelativePath(this->maskPath)));
  }

  if (this->config->available_values().size() > 0)
  {
    kwiver::vital::write_config_file(
      this->config, stdString(this->filePath));
  }
}
