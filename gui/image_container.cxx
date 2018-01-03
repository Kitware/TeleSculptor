/*ckwg +29
 * Copyright 2013-2016 by Kitware, Inc.
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

/**
 * \file
 * \brief OCV image_container implementation
 */

#include "image_container.h"

namespace kwiver {
namespace vtk {

/// Constructor - convert base image container to vtkImageData
image_container
::image_container(const kwiver::vital::image_container& image_cont)
{
}


/// The size of the image data in bytes
size_t
image_container
::size() const
{
  return data_->GetActualMemorySize();
}


/// Convert an OpenCV vtkImageData to a VITAL image
kwiver::vital::image
image_container
::vtk_to_vital(const vtkSmartPointer<vtkImageData> img)
{
  int dims[3];
  img->GetDimensions(dims);
  return kwiver::vital::image(dims[0], dims[1], dims[2]);
}


/// Convert an OpenCV vtkImageData type value to a kwiver::vital::image_pixel_traits
kwiver::vital::image_pixel_traits
image_container
::vtk_to_vital(int type)
{
  return kwiver::vital::image_pixel_traits();
}


/// Convert a VITAL image to an OpenCV vtkImageData
vtkSmartPointer<vtkImageData>
image_container
::vital_to_vtk(const kwiver::vital::image& img)
{
  std::cerr << "FRAME vital_to_vtk" << std::endl;
  auto retVal = vtkSmartPointer<vtkImageData>::New();
  retVal->SetDimensions(img.width(), img.height(), img.depth());
  return retVal;
}

/// Extract a vtkImageData from any image container
vtkSmartPointer<vtkImageData>
image_container_to_vtk(const kwiver::vital::image_container& img)
{
  return vtkSmartPointer<vtkImageData>::New();
}

} // end namespace vtk
} // end namespace kwiver
