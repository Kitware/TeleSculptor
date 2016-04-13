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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include <vital/config/config_block_io.h>

#include <qtStlUtil.h>

#include <QtGui/QApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>

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

    this->cameraPath = getPath(config, base, "output_krtd_dir");
    this->landmarks = getPath(config, base, "output_ply_file");
    this->tracks =
      getPath(config, base, "input_track_file", "output_tracks_file");

    this->DMvtp = getPath(config, base, "depthmaps_points_file");
    this->DMvts = getPath(config, base, "depthmaps_surfaces_file");

    // Read image list
    auto const& iflPath = config->get_value<std::string>("image_list_file");
    QFile ifl(base.filePath(qtString(iflPath)));
    if (!ifl.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      // TODO set error
      return false;
    }

    while (!ifl.atEnd())
    {
      auto const& line = ifl.readLine();
      if (!line.isEmpty())
      {
        // Strip '\n' and convert to full path
        auto const ll = line.length() - (line.endsWith('\n') ? 1 : 0);
        this->images.append(base.filePath(QString::fromLocal8Bit(line, ll)));
      }
    }

    return true;
  }
  catch (...)
  {
    // TODO set error
    return false;
  }
}
