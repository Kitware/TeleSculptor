/*ckwg +29
 * Copyright 2019 by Kitware, Inc.
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
 * \brief Implementation of PDAL point cloud writer
 */

#include "write_pdal.h"
#include <maptk/maptk_config.h>
#include <vital/logger/logger.h>
#include <vital/exceptions/base.h>
#include <vital/exceptions/io.h>

#ifdef MAPTK_USE_PDAL
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

#ifdef MAPTK_USE_PDAL

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
  kv::vector_2d offset_xy(0.0, 0.0);
  double offset_z = 0.0;
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
    offset_xy = lgcs.origin().location();
    offset_z = lgcs.origin_altitude();
    std::string srs_name = "EPSG:" + std::to_string(crs);
    pdal::SpatialReference srs;
    srs = pdal::SpatialReference(srs_name);
    view = std::make_shared<pdal::PointView>(table, srs);
  }

  unsigned int id = 0;
  for( unsigned int id=0; id < points.size(); ++id )
  {
    kv::vector_3d pt = points[id];
    pt[0] += offset_xy[0];
    pt[1] += offset_xy[1];
    pt[2] += offset_z;
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
