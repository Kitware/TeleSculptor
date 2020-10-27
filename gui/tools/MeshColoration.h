/*ckwg +29
 * Copyright 2016 by Kitware, SAS; Copyright 2017-2018 by Kitware, Inc.
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

#ifndef TELESCULPTOR_MESHCOLORATION_H_
#define TELESCULPTOR_MESHCOLORATION_H_

#include <QThread>
#include <arrows/vtk/mesh_coloration.h>

class MeshColoration : public QThread, public kwiver::arrows::vtk::mesh_coloration
{
  Q_OBJECT;

public:
  MeshColoration(kwiver::vital::config_block_sptr& videoConfig,
                 std::string const& videoPath,
                 kwiver::vital::config_block_sptr& maskConfig,
                 std::string const& maskPath,
                 kwiver::vital::camera_map_sptr& cameras);

  MeshColoration(MeshColoration const&) = delete;
  MeshColoration& operator=(MeshColoration const&) = delete;

  // Adds mean and median colors to 'Output' if averageColor or
  // adds an array of colors for each camera (frame) otherwise.
  void run() override;

  void report_progress_changed(const std::string& message, int percentage) override;
  
signals:
  void resultReady(MeshColoration* coloration);
  void progressChanged(QString, int);
};

#endif
