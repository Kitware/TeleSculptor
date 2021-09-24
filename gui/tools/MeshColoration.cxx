// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "MeshColoration.h"

// ----------------------------------------------------------------------------
MeshColoration::MeshColoration(
  kwiver::vital::config_block_sptr const& videoConfig,
  std::string const& videoPath,
  kwiver::vital::config_block_sptr const& maskConfig,
  std::string const& maskPath,
  kwiver::vital::camera_map_sptr const& cameras)
  : kwiver::arrows::vtk::mesh_coloration(
      videoConfig, videoPath, maskConfig, maskPath, cameras)
{
}

// ----------------------------------------------------------------------------
void MeshColoration::report_progress_changed(
  const std::string& message, int percentage)
{
  emit progressChanged(QString::fromStdString(message), percentage);
}

// ----------------------------------------------------------------------------
void MeshColoration::run()
{
  emit resultReady(this->colorize() ? this : nullptr);
}

