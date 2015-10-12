/*ckwg +29
 * Copyright 2015 by Kitware, Inc.
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

#include <vital/config/config_block_io.h>

#include <qtStlUtil.h>

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>

//-----------------------------------------------------------------------------
bool Project::read(QString const& path)
{
  auto const& base = QFileInfo(path).absoluteDir();

  try
  {
    // Load config file
    auto const& config = kwiver::vital::read_config_file(qPrintable(path));

    auto const& cameraPath = config->get_value<std::string>("output_krtd_dir");
    auto const& landmarks = config->get_value<std::string>("output_ply_file");
    auto const& tracks = config->get_value<std::string>("input_track_file");

    this->cameraPath = base.filePath(qtString(cameraPath));
    this->landmarks = base.filePath(qtString(landmarks));
    this->tracks = base.filePath(qtString(tracks));

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
