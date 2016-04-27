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

#include "Project.h"

#include <maptk/version.h>

#include <vital/config/config_block_io.h>

#include <qtStlUtil.h>

#include <QtGui/QApplication>
#include <QtCore/QDebug>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QTextStream>

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

//    this->DMvtp = getPath(config, base, "depthmaps_points_file");
//    this->DMvts = getPath(config, base, "depthmaps_surfaces_file");
//    this->DMvti = getPath(config, base, "depthmaps_images_file");
//    this->DMvert = getPath(config, base, "depthmaps_vertices_file");

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

    //Read depthmap points list
    std::string vtpFilePath;
    try {
      vtpFilePath = config->get_value<std::string>("depthmaps_points_file");
    } catch (...) {
    }

    if (!vtpFilePath.empty()) {
      QFile vtpFile(base.filePath(qtString(vtpFilePath)));
      if (vtpFile.open(QIODevice::ReadOnly | QIODevice::Text))
      {
        QTextStream vtpStream(&vtpFile);
        int frameNum;
        QString fileName;
        while (!vtpStream.atEnd())
        {
          vtpStream >> frameNum >> fileName;
          if (!DMvtp.contains(frameNum))
          {
            DMvtp.insert(frameNum,fileName);
          }
        }
      }
      vtpFile.close();
    }

    //Read depthmap surfaces list
    std::string vtsFilePath;
    try {

      vtsFilePath = config->get_value<std::string>("depthmaps_surfaces_file");
    } catch (...) {
    }

    if (!vtpFilePath.empty()) {
      QFile vtsFile(base.filePath(qtString(vtsFilePath)));
      if (vtsFile.open(QIODevice::ReadOnly | QIODevice::Text))
      {
        QTextStream vtsStream(&vtsFile);
        int frameNum;
        QString fileName;
        while (!vtsStream.atEnd())
        {
          vtsStream >> frameNum >> fileName;
          if (!DMvts.contains(frameNum))
          {
            DMvts.insert(frameNum,fileName);
          }
        }
      }
      vtsFile.close();
    }

    //Read depthmap images list
    std::string vtiFilePath;
    try {

      vtiFilePath = config->get_value<std::string>("depthmaps_images_file");
    } catch (...) {
    }

    if (!vtiFilePath.empty()) {
      QFile vtiFile(base.filePath(qtString(vtiFilePath)));
      if (vtiFile.open(QIODevice::ReadOnly | QIODevice::Text))
      {
        QTextStream vtiStream(&vtiFile);
        int frameNum;
        QString fileName;
        while (!vtiStream.atEnd())
        {
          vtiStream >> frameNum >> fileName;
          if (!DMvti.contains(frameNum))
          {
            DMvti.insert(frameNum,fileName);
          }
        }
      }
      vtiFile.close();
    }

    //Read depthmap vertices list
    std::string vertFilePath;
    try {

      vertFilePath = config->get_value<std::string>("depthmaps_vertices_file");
    } catch (...) {
    }
    if (!vertFilePath.empty()) {
      QFile vertFile(base.filePath(qtString(vertFilePath)));
      if (vertFile.open(QIODevice::ReadOnly | QIODevice::Text))
      {
        QTextStream vertStream(&vertFile);
        int frameNum;
        QString fileName;
        while (!vertStream.atEnd())
        {
          vertStream >> frameNum >> fileName;
          if (!DMvert.contains(frameNum))
          {
            DMvert.insert(frameNum,fileName);
          }
        }
      }
      vertFile.close();
    }

    std::cout << "after ifs" << std::endl;


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
