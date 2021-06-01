// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Definition for PDAL point cloud writer
 */

#ifndef MAPTK_WRITE_PDAL_H_
#define MAPTK_WRITE_PDAL_H_

#include <maptk/maptk_export.h>

#include <vital/types/local_geo_cs.h>
#include <vital/types/landmark_map.h>

namespace kwiver {
namespace maptk {

/// Write landmarks to a file with PDAL
MAPTK_EXPORT
void
write_pdal(vital::path_t const& filename,
           vital::local_geo_cs const& lgcs,
           vital::landmark_map_sptr const& landmarks);

/// Write point cloud to a file with PDAL
MAPTK_EXPORT
void
write_pdal(vital::path_t const& filename,
           vital::local_geo_cs const& lgcs,
           std::vector<vital::vector_3d> const& points,
           std::vector<vital::rgb_color> const& colors = {});

} // end namespace maptk
} // end namespace kwiver

#endif // MAPTK_WRITE_PDAL_H_
