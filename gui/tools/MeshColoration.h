// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_MESHCOLORATION_H_
#define TELESCULPTOR_MESHCOLORATION_H_

#include <QThread>
#include <arrows/vtk/mesh_coloration.h>

class MeshColoration : public QThread,
                       public kwiver::arrows::vtk::mesh_coloration
{
  Q_OBJECT;

public:
  MeshColoration(kwiver::vital::config_block_sptr const& videoConfig,
                 std::string const& videoPath,
                 kwiver::vital::config_block_sptr const& maskConfig,
                 std::string const& maskPath,
                 kwiver::vital::camera_map_sptr const& cameras);

  MeshColoration(MeshColoration const&) = delete;
  MeshColoration& operator=(MeshColoration const&) = delete;

  // Adds mean and median colors to 'Output' if averageColor or
  // adds an array of colors for each camera (frame) otherwise.
  void run() override;

  void report_progress_changed(
    const std::string& message, int percentage) override;

signals:
  void resultReady(MeshColoration* coloration);
  void progressChanged(QString, int);
};

#endif
