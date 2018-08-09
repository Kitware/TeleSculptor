/*ckwg +29
 * Copyright 2016 by Kitware, SAS; Copyright 2017 by Kitware, Inc.
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

#ifndef MAPTK_MESHCOLORATION_H_
#define MAPTK_MESHCOLORATION_H_

// KWIVER includes
#include <vital/algo/video_input.h>
#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>

// VTK Class
class vtkPolyData;

// Project class
class ReconstructionData;

#include <string>
#include <vector>

class MeshColoration
{
public:
  MeshColoration();
  MeshColoration(vtkPolyData* mesh, std::string frameList, std::string krtdFolder);
  MeshColoration(vtkPolyData* mesh,
                 kwiver::vital::config_block_sptr& config,
                 std::string videoPath,
                 kwiver::vital::camera_map_sptr& cameras);
  ~MeshColoration();

  // SETTER
  void SetInput(vtkPolyData* mesh);
  void SetFrameSampling(int sample);

  // GETTER
  vtkPolyData* GetOutput();

  // Functions
  bool ProcessColoration(std::string currentVtiPath ="");
  void initializeDataList(std::string currentVtiPath ="");

protected:
  // Attributes
  vtkPolyData* OutputMesh;
  int Sampling;
  std::vector<ReconstructionData*> DataList;
  std::vector<std::string> frameList;
  std::vector<std::string> krtdFolder;

  std::string videoPath;
  kwiver::vital::algo::video_input_sptr videoReader;
  kwiver::vital::camera_map_sptr cameras;
};

#endif
