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

// KWIVER includes
#include <vital/algo/video_input.h>
#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>
#include <vital/types/camera_perspective.h>

// VTK Class
class vtkPolyData;

#include <string>
#include <vector>

// Qt classes
#include <QThread>

class MeshColoration : public QThread
{
  Q_OBJECT;

public:
  MeshColoration();
  MeshColoration(kwiver::vital::config_block_sptr& config,
                 std::string const& videoPath,
                 kwiver::vital::camera_map_sptr& cameras);
  ~MeshColoration();

  MeshColoration(MeshColoration const&) = delete;
  MeshColoration& operator=(MeshColoration const&) = delete;

  // Input mesh.
  void SetInput(vtkPolyData* mesh);
  vtkPolyData* GetInput();
  // Output mesh
  void SetOutput(vtkPolyData* mesh);
  vtkPolyData* GetOutput();
  void SetFrameSampling(int sample);
  void SetFrame(int frame)
  { this->Frame = frame;}
  void SetAverageColor(bool averageColor)
  { this->AverageColor = averageColor;}

  // Adds mean and median colors to 'Output' if averageColor or
  // adds an array of colors for each camera (frame) otherwise.
  void run() override;

signals:
  void resultReady(MeshColoration* coloration);

protected:
  void initializeDataList(int frameId);

protected:
  // input mesh
  vtkPolyData* Input;
  vtkPolyData* Output;
  int Sampling;
  int Frame;
  bool AverageColor;
  bool Error;

  struct ColorationData
  {
    ColorationData(kwiver::vital::image_of<uint8_t> image,
                   kwiver::vital::camera_perspective_sptr camera_ptr,
                   kwiver::vital::frame_id_t frame) :
      Image(image), Camera_ptr(camera_ptr), Frame(frame)
    {}
    kwiver::vital::image_of<uint8_t> Image;
    kwiver::vital::camera_perspective_sptr Camera_ptr;
    kwiver::vital::frame_id_t Frame;
  };
  std::vector<ColorationData> DataList;

  std::string videoPath;
  kwiver::vital::algo::video_input_sptr videoReader;
  kwiver::vital::camera_map_sptr cameras;
};

#endif
