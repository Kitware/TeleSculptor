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

/**
 * \file
 * \brief projected_track_set implementation
 */

#include "undistort_frame.h"

#include <vtkDataArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

#include <kwiversys/SystemTools.hxx>
namespace kwiver {
namespace maptk {

using namespace kwiver::vital;

/// Use the cameras to project the landmarks back into their images.
vtkSmartPointer<vtkImageData> undistortFrame(std::string const framePath,
                                             camera *cam)
{
  //Read the frame into an imageData
  auto const reader = vtkImageReader2Factory::CreateImageReader2(framePath.c_str());

  reader->SetFileName(framePath.c_str());
  reader->Update();

  auto const originalFrame = reader->GetOutput();

  //Create an empty imageData

  vtkSmartPointer<vtkImageData> undistortedFrame = vtkSmartPointer<vtkImageData>::New();

  undistortedFrame->SetSpacing(originalFrame->GetSpacing());
  undistortedFrame->SetOrigin(originalFrame->GetOrigin());
  undistortedFrame->SetDimensions(originalFrame->GetDimensions());

  //Create an empty array for the undistorted values

  vtkNew<vtkUnsignedCharArray> undistortedValues;

  undistortedValues->SetName("Undistorted Values");
  undistortedValues->SetNumberOfComponents(3);
  undistortedValues->SetNumberOfTuples(originalFrame->GetPointData()->
                                       GetScalars()->GetNumberOfTuples());

  //Get old image as array
  vtkDataArray* scalars = originalFrame->GetPointData()->GetScalars();

  double originalPixel[3], undistortedCoord[3];
  vtkIdType undistortedPixelId;

  //Create a perfect camera for mapping
  simple_camera* perfectCam = new simple_camera(*cam);

  simple_camera_intrinsics* perfectIntrinsics =
      new simple_camera_intrinsics(*cam->intrinsics());

  Eigen::VectorXd noDist(1);
  noDist(0) = 0;

  perfectIntrinsics->set_dist_coeffs(noDist);

  camera_intrinsics_sptr perfectIntSptr(perfectIntrinsics);

  perfectCam->set_intrinsics(perfectIntSptr);

  vector_2d normalizedCoord, newCoord;

  //Undistort each point and save it in the new array
  for (vtkIdType idPixel = 0; idPixel < originalFrame->GetNumberOfPoints(); ++idPixel)
  {
    originalFrame->GetPoint(idPixel,originalPixel);

    normalizedCoord = cam->intrinsics()->unmap(vector_2d(originalPixel[0],
                                               originalPixel[1]));

    newCoord = perfectCam->intrinsics()->map(normalizedCoord);

    undistortedCoord[0] = newCoord[0];
    undistortedCoord[1] = newCoord[1];
    undistortedCoord[2] = originalPixel[2];

    //Get id of undistorted pixel in original frame

    undistortedPixelId = originalFrame->FindPoint(undistortedCoord);

    undistortedValues->SetTuple(idPixel,
                                scalars->GetTuple(undistortedPixelId));
  }

  undistortedFrame->GetPointData()->SetScalars(undistortedValues.Get());

  return undistortedFrame.GetPointer();
}

void undistortFrames(const std::vector<std::string> framePathList,
                     const std::vector<vital::camera_sptr> camList,
                     const std::string outputDir)
{
  vtkNew<vtkPNGWriter> imageWriter;
  vtkSmartPointer<vtkImageData> undistortedFrame;

  for (int i = 0; i < framePathList.size(); ++i)
  {

    std::string originalFramePath = framePathList[i];
    auto const& camera = camList[i];

    std::string filename = kwiversys::SystemTools::ConvertToOutputPath(outputDir
                          + "/"
                          + kwiversys::SystemTools::GetFilenameName(originalFramePath));

    std::cerr << "Undistorting " << filename.c_str() << "... ";

    undistortedFrame = undistortFrame(originalFramePath, camera.get());

    imageWriter->SetInputData(undistortedFrame);
    imageWriter->SetFileName(filename.c_str());

    std::cerr << "Done." << std::endl;

    imageWriter->Write();
  }
}

} // end namespace maptk
} // end namespace kwiver
