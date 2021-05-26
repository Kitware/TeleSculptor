// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

/**
 * \file
 * \brief Implementation of PDAL point cloud writer
 */

#include "write_pdal.h"
#include <maptk/maptk_config.h>
#include <vital/logger/logger.h>
#include <vital/exceptions/base.h>
#include <vital/exceptions/io.h>

#ifdef TELESCULPTOR_USE_PDAL
#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>

#include <io/BufferReader.hpp>
#endif

namespace kwiver {
namespace maptk {

/// Write landmarks to a file with PDAL
void
  write_pdal(vital::path_t const& filename,
             vital::local_geo_cs const& lgcs,
             vital::landmark_map_sptr const& landmarks)
{
  std::vector<vital::vector_3d> points;
  std::vector<vital::rgb_color> colors;
  points.reserve(landmarks->size());
  colors.reserve(landmarks->size());
  for (auto lm : landmarks->landmarks())
  {
    points.push_back(lm.second->loc());
    colors.push_back(lm.second->color());
  }
  write_pdal(filename, lgcs, points, colors);
}

/// Write point cloud to a file with PDAL
void
write_pdal(vital::path_t const& filename,
           vital::local_geo_cs const& lgcs,
           std::vector<vital::vector_3d> const& points,
           std::vector<vital::rgb_color> const& colors)
{
  namespace kv = kwiver::vital;
  kv::logger_handle_t logger( kv::get_logger( "write_pdal" ) );

#ifdef TELESCULPTOR_USE_PDAL

  if( !colors.empty() && colors.size() != points.size() )
  {
    throw vital::invalid_value("write_pdal: number of colors provided does "
                               "not match the number of points");
  }

  pdal::Options options;
  options.add("filename", filename);
  options.add("system_id", "TeleSculptor");
  options.add("offset_x", "auto");
  options.add("offset_y", "auto");
  options.add("offset_z", "auto");

  pdal::PointTable table;
  table.layout()->registerDim(pdal::Dimension::Id::X);
  table.layout()->registerDim(pdal::Dimension::Id::Y);
  table.layout()->registerDim(pdal::Dimension::Id::Z);
  if( !colors.empty() )
  {
    table.layout()->registerDim(pdal::Dimension::Id::Red);
    table.layout()->registerDim(pdal::Dimension::Id::Green);
    table.layout()->registerDim(pdal::Dimension::Id::Blue);
  }

  int crs = lgcs.origin().crs();
  kv::vector_3d offset(0.0, 0.0, 0.0);
  pdal::PointViewPtr view;
  // handle special cases of non-geographic coordinates
  if( crs < 0 )
  {
    view = std::make_shared<pdal::PointView>(table);
    options.add("scale_x", 1e-4);
    options.add("scale_y", 1e-4);
    options.add("scale_z", 1e-4);
  }
  else
  {
    offset = lgcs.origin().location();
    std::string srs_name = "EPSG:" + std::to_string(crs);
    pdal::SpatialReference srs;
    srs = pdal::SpatialReference(srs_name);
    view = std::make_shared<pdal::PointView>(table, srs);
  }

  for( unsigned int id=0; id < points.size(); ++id )
  {
    kv::vector_3d pt = points[id] + offset;
    view->setField(pdal::Dimension::Id::X, id, pt.x());
    view->setField(pdal::Dimension::Id::Y, id, pt.y());
    view->setField(pdal::Dimension::Id::Z, id, pt.z());
    if (!colors.empty())
    {
      kv::rgb_color const& rgb = colors[id];
      view->setField(pdal::Dimension::Id::Red, id, rgb.r);
      view->setField(pdal::Dimension::Id::Green, id, rgb.g);
      view->setField(pdal::Dimension::Id::Blue, id, rgb.b);
    }
  }

  pdal::BufferReader reader;
  reader.addView(view);

  pdal::StageFactory factory;
  pdal::Stage *writer = factory.createStage("writers.las");

  writer->setInput(reader);
  writer->setOptions(options);
  writer->prepare(table);
  writer->execute(table);

#else

  throw vital::file_write_exception(filename,
                                    "TeleSculptor was not compiled with PDAL, "
                                    "cannot write LAS.");
#endif
}

} // end namespace maptk
} // end namespace kwiver
