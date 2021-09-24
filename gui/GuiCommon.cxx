// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "GuiCommon.h"

#include <maptk/version.h>

#include <vital/io/metadata_io.h>

#include <kwiversys/SystemTools.hxx>

#include <vtkImageFlip.h>
#include <vtkImageImport.h>

#include <QDir>
#include <QApplication>

//-----------------------------------------------------------------------------
std::string
frameName(kwiver::vital::frame_id_t frame,
          kwiver::vital::metadata_map const& mdm)
{
  using kwiver::vital::basename_from_metadata;
  auto mdv = mdm.get_vector(frame);
  return frameName(frame, mdv);
}

//-----------------------------------------------------------------------------
std::string
frameName(kwiver::vital::frame_id_t frame,
          kwiver::vital::metadata_vector const& mdv)
{
  using kwiver::vital::basename_from_metadata;
  for (auto const& md : mdv)
  {
    if (md->has(kwiver::vital::VITAL_META_IMAGE_URI) ||
        md->has(kwiver::vital::VITAL_META_VIDEO_URI))
    {
      return basename_from_metadata(md, frame);
    }
  }
  return basename_from_metadata(nullptr, frame);
}

//----------------------------------------------------------------------------
std::string
frameName(kwiver::vital::frame_id_t frame,
          kwiver::vital::metadata_sptr md)
{
  return kwiver::vital::basename_from_metadata(md, frame);
}

//----------------------------------------------------------------------------
kwiver::vital::config_block_sptr readConfig(std::string const& name)
{
  try
  {
    using kwiver::vital::read_config_file;

    auto const exeDir = QDir(QApplication::applicationDirPath());
    auto const prefix = stdString(exeDir.absoluteFilePath(".."));
    return read_config_file(name, "telesculptor", TELESCULPTOR_VERSION, prefix);
  }
  catch (...)
  {
    return {};
  }
}

//----------------------------------------------------------------------------
// find the full path to the first matching file on the config search path
kwiver::vital::path_t findConfig(std::string const& name)
{
  try
  {
    using kwiver::vital::application_config_file_paths;

    auto const exeDir = QDir(QApplication::applicationDirPath());
    auto const prefix = stdString(exeDir.absoluteFilePath(".."));
    auto const& search_paths =
    application_config_file_paths("telesculptor", TELESCULPTOR_VERSION, prefix);

    for (auto const& search_path : search_paths)
    {
      auto const& config_path = search_path + "/" + name;

      if (kwiversys::SystemTools::FileExists(config_path) &&
          !kwiversys::SystemTools::FileIsDirectory(config_path))
      {
          return config_path;
      }
    }
  }
  catch (...)
  {
    return "";
  }
  return "";
}

//----------------------------------------------------------------------------
vtkSmartPointer<vtkImageData> vitalToVtkImage(kwiver::vital::image const& img)
{
  kwiver::vital::image frameImg;
  // If image is interlaced it is already compatible with VTK
  if (img.d_step() == 1)
  {
    frameImg = img;
  }
  // Otherwise we need a deep copy to get it to be interlaced
  else
  {
    // create an interlaced image of the same dimensions and type
    frameImg = kwiver::vital::image(img.width(), img.height(), img.depth(),
                                    true, img.pixel_traits());
    frameImg.copy_from(img);
  }

  auto imgTraits = frameImg.pixel_traits();

  // Get the image type
  int imageType = VTK_VOID;
  switch (imgTraits.type)
  {
    case kwiver::vital::image_pixel_traits::UNSIGNED:
      imageType = VTK_UNSIGNED_CHAR;
      break;
    case kwiver::vital::image_pixel_traits::SIGNED:
      imageType = VTK_SIGNED_CHAR;
      break;
    case kwiver::vital::image_pixel_traits::FLOAT:
      imageType = VTK_FLOAT;
      break;
    default:
      imageType = VTK_VOID;
      break;
    // TODO: exception or error/warning message?
  }

	// convert to vtkFrameData
    vtkSmartPointer<vtkImageImport> imageImport =
        vtkSmartPointer<vtkImageImport>::New();
    imageImport->SetDataScalarType(imageType);
    imageImport->SetNumberOfScalarComponents(static_cast<int>(frameImg.depth()));
    imageImport->SetWholeExtent(0, static_cast<int>(frameImg.width()) - 1,
                                0, static_cast<int>(frameImg.height()) - 1, 0, 0);
    imageImport->SetDataExtentToWholeExtent();
    imageImport->SetImportVoidPointer(frameImg.first_pixel());
    imageImport->Update();

    // Flip image so it has the correct axis for VTK
    vtkSmartPointer<vtkImageFlip> flipFilter =
        vtkSmartPointer<vtkImageFlip>::New();
    flipFilter->SetFilteredAxis(1); // flip x axis
    flipFilter->SetInputConnection(imageImport->GetOutputPort());
    flipFilter->Update();

    return flipFilter->GetOutput();
}
