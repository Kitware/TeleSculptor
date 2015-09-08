/*ckwg +5
 * Copyright 2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

// VTK includes.
#include <vtkCamera.h>
#include <vtkCameraActor.h>
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
  vtkPolyData* cameraPD = vtkPolyData::New();

  vtkPoints* cameraPoints = vtkPoints::New();
  cameraPoints->Allocate(files.size());
  cameraPD->SetPoints(cameraPoints);
  cameraPoints->FastDelete();

  vtkDoubleArray* normals = vtkDoubleArray::New();
  normals->SetNumberOfComponents(3);
  cameraPD->GetPointData()->SetNormals(normals);
  normals->FastDelete();

  auto cone = vtkConeSource::New();
  cone->SetHeight(10);
  cone->SetCenter(5, 0, 0);

  auto glyph = vtkGlyph3D::New();
  glyph->SetInputData(cameraPD);
  cameraPD->FastDelete();

  glyph->SetSourceConnection(cone->GetOutputPort());
  cone->FastDelete();
  glyph->SetVectorModeToUseNormal();

  auto cameraMapper = vtkPolyDataMapper::New();
  cameraMapper->SetInputConnection(glyph->GetOutputPort());
  glyph->FastDelete();

  auto cameraActor = vtkActor::New();
  cameraActor->SetMapper(cameraMapper);
  cameraMapper->FastDelete();
  
  renderer->AddActor(cameraActor);
  cameraActor->FastDelete();

  vtkPolyData* cameraUpPD = vtkPolyData::New();
  cameraUpPD->SetPoints(cameraPoints);

  auto upCone = vtkConeSource::New();
  upCone->SetHeight(4);
  upCone->SetCenter(2, 0, 0);

  auto upGlyph = vtkGlyph3D::New();
  upGlyph->SetInputData(cameraUpPD);
  cameraUpPD->FastDelete();

  upGlyph->SetSourceConnection(upCone->GetOutputPort());
  upCone->FastDelete();
  upGlyph->SetVectorModeToUseNormal();

  vtkDoubleArray* upNormals = vtkDoubleArray::New();
  upNormals->SetNumberOfComponents(3);
  cameraUpPD->GetPointData()->SetNormals(upNormals);
  upNormals->FastDelete();

  auto cameraUpMapper = vtkPolyDataMapper::New();
  cameraUpMapper->SetInputConnection(upGlyph->GetOutputPort());
  upGlyph->FastDelete();

  auto cameraUpActor = vtkActor::New();
  cameraUpActor->SetMapper(cameraUpMapper);
  cameraUpMapper->FastDelete();

  renderer->AddActor(cameraUpActor);
  cameraUpActor->FastDelete();
  cameraUpActor->GetProperty()->SetColor(1, 0, 0);
  
  // loop over cameras (krtd files), and add them to the scene
  for (int i = 0; i < files.size(); i++)
    {
    auto krtdCamera = maptk::read_krtd_file(files[i]);
    auto rotationMatrix = maptk::matrix_3x3d(krtdCamera.get_rotation());
 
#if 0
    // Based on D Gobbi answer(s) at
    // http://vtk.1045678.n5.nabble.com/Question-on-manual-configuration-of-VTK-camera-td5059478.html
    Eigen::MatrixXd modelViewA(rotationMatrix.rows(), rotationMatrix.cols()+1);
    modelViewA << rotationMatrix, krtdCamera.get_translation();

    Eigen::RowVector4d homog(0, 0, 0, 1);
    Eigen::MatrixXd modelView(modelViewA.rows() + 1, modelViewA.cols());
    modelView << modelViewA, homog;

    auto inverseMatrix = modelView.inverse();
    if (0)
      {
      auto center = inverseMatrix.col(3);
      auto upVector = inverseMatrix.col(1);
      auto viewVector = inverseMatrix.col(2); // like ksimek, view vector opposite of indicated

      cameraPoints->InsertNextPoint(center(0), center(1), center(2));
      normals->InsertNextTuple3(viewVector(0), viewVector(1), viewVector(2));
      upNormals->InsertNextTuple3(upVector(0), upVector(1), upVector(2));
      }
    else
      {
      auto transform = vtkTransform::New();
      auto matrix = vtkMatrix4x4::New();
      for (int i = 0; i < 4; i++)
        {
        //auto row = inverseMatrix.row
        for (int j = 0; j < 4; j++)
          {
          matrix->SetElement(i, j, inverseMatrix.row(i)(j));
          }
        }
      transform->SetMatrix(matrix);
      matrix->FastDelete();

      double pos[3] = { 0.0, 0.0, 0.0 };
      double zvec[3] = { 0.0, 0.0, 1.0 };
      double yvec[3] = { 0.0, 1.0, 0.0 };

      transform->TransformPoint(pos, pos);
      transform->TransformVector(zvec, zvec);
      transform->TransformVector(yvec, yvec);

      cameraPoints->InsertNextPoint(pos);
      normals->InsertNextTuple(zvec);
      upNormals->InsertNextTuple(yvec);

      transform->Delete();
      }
#else
    // Based on http://ksimek.github.io/2012/08/22/extrinsic/
    auto center = krtdCamera.get_center().transpose();
    auto upVector = rotationMatrix.row(1);
    // viewVector here is opposite of that indicated
    auto viewVector = rotationMatrix.row(2);

    cameraPoints->InsertNextPoint(center(0), center(1), center(2));
    normals->InsertNextTuple3(viewVector(0), viewVector(1), viewVector(2));
    upNormals->InsertNextTuple3(upVector(0), upVector(1), upVector(2));
#endif
    }

  renWin->Render();
  iren->Start();

  renWin->Delete();
  return 0;
}
