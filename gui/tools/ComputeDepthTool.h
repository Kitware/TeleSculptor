/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
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

#ifndef TELESCULPTOR_COMPUTEDEPTHTOOL_H_
#define TELESCULPTOR_COMPUTEDEPTHTOOL_H_

#include "AbstractTool.h"

class ComputeDepthToolPrivate;

class ComputeDepthTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit ComputeDepthTool(QObject* parent = nullptr);
  ~ComputeDepthTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

  bool callback_handler(kwiver::vital::image_container_sptr depth,
                        std::string const& status,
                        unsigned int percent_complete,
                        kwiver::vital::image_container_sptr uncertainty);

  /// handler for callback on image gathering status
  bool gather_status_handler(unsigned int curr_frame,
                             unsigned int num_frames);

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(ComputeDepthTool)
  QTE_DECLARE_PRIVATE(ComputeDepthTool)
  QTE_DISABLE_COPY(ComputeDepthTool)
};

///Convert a kwiver depth map to a colored vtk image with optional mask
vtkSmartPointer<vtkImageData>
depth_to_vtk(const kwiver::vital::image_of<double>& depth_img,
             const kwiver::vital::image_of<unsigned char>& color_img,
             int i0, int ni, int j0, int nj,
             const kwiver::vital::image_of<double>& uncertainty_img = {},
             const kwiver::vital::image_of<unsigned char>& mask_img = {});

#endif
