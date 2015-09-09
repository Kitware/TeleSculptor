/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

// VTK includes.
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkFrustumSource.h>
#include <vtkPlanes.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>

#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkGlyph3D.h>
#include <vtkConeSource.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>

#include <vtkAppendPolyData.h>

#include <vtksys/Glob.hxx>

// STL includes.
#include <stdlib.h>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <string>

#include <maptk/eigen_io.h>
#include <maptk/camera.h>
#include <maptk/camera_io.h>
#include <maptk/matrix.h>
#include <maptk/landmark_map_io.h>

#ifdef _WIN32
#define strcasecmp _stricmp
#endif

int main(int argc, char** argv)
{
  if (argc < 3)
    {
    cout << "Usage: " << argv[0] << " krtd-directory ply-file" << endl;
    return 1;
    }

  // Check for camera files
  std::string krtdPath = argv[1];
  vtksys::Glob glob;
  glob.FindFiles(krtdPath + "/*.krtd");
  std::vector<std::string>& files = glob.GetFiles();
  if (files.empty())
    {
    cout << "Error: No krtd files found!" << endl;
    return 1;
    }

  // Gerneal rendering setup
  vtkRenderer *renderer = vtkRenderer::New();
  vtkRenderWindow *renWin = vtkRenderWindow::New();
  renWin->AddRenderer(renderer);
  renderer->FastDelete();

  vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
  iren->SetRenderWindow(renWin);
  iren->FastDelete();

  // Handle the landmarks
  auto landmarks = maptk::read_ply_file(argv[2]);
  vtkPolyData* pD = vtkPolyData::New();

  vtkPoints* points = vtkPoints::New();
  points->SetNumberOfPoints(landmarks->size());
  pD->SetPoints(points);
  points->FastDelete();

  auto glyphLandmarks = vtkGlyph3D::New();
  glyphLandmarks->SetInputData(pD);
  pD->FastDelete();

  auto sphere = vtkSphereSource::New();
  sphere->SetRadius(0.01);
  glyphLandmarks->SetSourceConnection(sphere->GetOutputPort());
  sphere->FastDelete();

  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputConnection(glyphLandmarks->GetOutputPort());
  glyphLandmarks->FastDelete();

  auto landmarkMap = landmarks->landmarks();
  auto iter = landmarkMap.begin();
  for (int i = 0; iter != landmarkMap.end(); iter++, i++)
    {
    points->SetPoint(i, iter->second->loc().data());
    }

  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);
  mapper->FastDelete();
  
  renderer->AddActor(actor);
  actor->FastDelete();


  // Now process the cameras
  vtkAppendPolyData* cameraAppend = vtkAppendPolyData::New();

  auto cameraMapper = vtkPolyDataMapper::New();
  cameraMapper->SetInputConnection(cameraAppend->GetOutputPort());
  cameraAppend->FastDelete();

  auto cameraActor = vtkActor::New();
  cameraActor->SetMapper(cameraMapper);
  cameraMapper->FastDelete();
  
  renderer->AddActor(cameraActor);
  cameraActor->FastDelete();
  cameraActor->GetProperty()->SetRepresentationToWireframe();

  auto camera = vtkCamera::New();
  double focalLength = 1.0;
  double farClipDistance = 5;
  double aspectRatio = 1;

  // loop over cameras (krtd files), and add them to the scene
  for (int i = 0; i < files.size(); i += 20)
    {
    auto krtdCamera = maptk::read_krtd_file(files[i]);
    auto rotationMatrix = maptk::matrix_3x3d(krtdCamera.get_rotation());

    // Based on http://ksimek.github.io/2012/08/22/extrinsic/
    auto center = krtdCamera.get_center().transpose();
    // viewVector and upVector here are opposite of that indicated, or so it would seem
    auto upVector = -rotationMatrix.row(1);
    auto viewVector = rotationMatrix.row(2);
    auto focalPt = center + viewVector * focalLength;

    auto planes = vtkPlanes::New();
    auto frustrum = vtkFrustumSource::New();
    frustrum->SetPlanes(planes);
    planes->FastDelete();

    camera->SetPosition(center(0), center(1), center(2));
    camera->SetFocalPoint(focalPt(0), focalPt(1), focalPt(2));
    camera->SetViewUp(upVector(0), upVector(1), upVector(2));
    camera->SetClippingRange(0.01, farClipDistance);

    double planeCoeffs[24];
    camera->GetFrustumPlanes(aspectRatio, planeCoeffs);
    planes->SetFrustumPlanes(planeCoeffs);
    frustrum->SetShowLines(false);
    frustrum->Update();

    cameraAppend->AddInputConnection(frustrum->GetOutputPort());
    frustrum->FastDelete();
    }

  camera->Delete();

  renWin->Render();
  iren->Start();

  renWin->Delete();
  return 0;
}
