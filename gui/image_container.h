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
 * \brief VTK image_container inteface
 */

#ifndef MAPTK_VTK_IMAGE_CONTAINER_H_
#define MAPTK_VTK_IMAGE_CONTAINER_H_

#include <arrows/ocv/kwiver_algo_ocv_export.h>

#include <vital/types/image_container.h>

#include <vtkImageData.h>
#include <vtkSmartPointer.h>

namespace kwiver {
namespace vtk {

/// This image container wraps a vtkImageData
class KWIVER_ALGO_OCV_EXPORT image_container
  : public kwiver::vital::image_container
{
public:

  /// Constructor - from vtkImageData
  explicit image_container(const vtkImageData& d) {}

  /// Constructor - convert kwiver image to vtkImageData
  explicit image_container(const kwiver::vital::image& vital_image) {}
  // : data_(vital_to_vtk(vital_image)) {}

  /// Constructor - convert base image container to vtkImageData
  explicit image_container(const kwiver::vital::image_container& image_cont);

  /// Copy Constructor
  image_container(const image_container& other) {}
  // : data_(other.data_) {}

  /// The size of the image data in bytes
  /**
   * This size includes all allocated image memory,
   * which could be larger than width*height*depth.
   */
  virtual size_t size() const;

  /// The width of the image in pixels
  virtual size_t width() const { return dims_[0]; }

  /// The height of the image in pixels
  virtual size_t height() const { return dims_[1]; }

  /// The depth (or number of channels) of the image
  virtual size_t depth() const { return dims_[2]; }

  /// Get and in-memory image class to access the data
  virtual kwiver::vital::image get_image() const { return vtk_to_vital(data_); }

  /// Access the underlying vtkImageData data structure
  // TODO: vtkImageData get_Mat() const { return data_; }

  /// Convert an OpenCV vtkImageData to a VITAL image
  static kwiver::vital::image vtk_to_vital(const vtkSmartPointer<vtkImageData> img);

  /// Convert an OpenCV vtkImageData type value to a kwiver::vital::image_pixel_traits
  static kwiver::vital::image_pixel_traits vtk_to_vital(int type);

  /// Convert a VITAL image to an OpenCV vtkImageData
  static vtkSmartPointer<vtkImageData> vital_to_vtk(const kwiver::vital::image& img);

  /// Convert a kwiver::vital::image_pixel_traits to an OpenCV vtkImageData type integer
  static int vital_to_vtk(const kwiver::vital::image_pixel_traits& pt);

protected:
  /// image data
  vtkSmartPointer<vtkImageData> data_;
  int dims_[3];
};


/// Extract a vtkImageData from any image container
/**
 * If \a img is actually an arrows::ocv::image_container then
 * return the underlying vtkImageData.  Otherwise, convert the image data
 * to cv:Mat by shallow copy (if possible) or deep copy as a last resort.
 *
 * \param img Image container to convert to cv::mat
 */
KWIVER_ALGO_OCV_EXPORT vtkSmartPointer<vtkImageData> image_container_to_vtk(const kwiver::vital::image_container& img);

} // end namespace vtk
} // end namespace kwiver

#endif // MAPTK_VTK_IMAGE_CONTAINER_H_
