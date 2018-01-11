/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
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
 *  * Neither the name Kitware, Inc. nor the names of any contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
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

#include "TrackFeaturesSprokitTool.h"

#include <fstream>
#include <sstream>
#include <maptk/colorize.h>
#include <maptk/version.h>

#include <vital/algo/image_io.h>
#include <vital/algo/convert_image.h>
#include <vital/algo/track_features.h>

#include <vital/config/config_block_io.h>
#include <vital/types/metadata.h>
#include <vital/types/metadata_traits.h>

#include <qtStlUtil.h>

#include <QtGui/QApplication>
#include <QtGui/QMessageBox>

#include <QtCore/QDir>

#include <sprokit/pipeline/pipeline.h>
#include <sprokit/processes/kwiver_type_traits.h>
#include <sprokit/processes/adapters/embedded_pipeline.h>
#include <sprokit/pipeline_util/literal_pipeline.h>

using kwiver::vital::algo::image_io;
using kwiver::vital::algo::image_io_sptr;
using kwiver::vital::algo::convert_image;
using kwiver::vital::algo::convert_image_sptr;
using kwiver::vital::algo::track_features;
using kwiver::vital::algo::track_features_sptr;

namespace
{
static char const* const BLOCK_IR = "image_reader";
static char const* const BLOCK_CI = "image_converter";
static char const* const BLOCK_TF = "feature_tracker";

//-----------------------------------------------------------------------------
kwiver::vital::config_block_sptr readConfig(std::string const& name)
{
  try
  {
    using kwiver::vital::read_config_file;

    auto const exeDir = QDir(QApplication::applicationDirPath());
    auto const prefix = stdString(exeDir.absoluteFilePath(".."));
    return read_config_file(name, "maptk", MAPTK_VERSION, prefix);
  }
  catch (...)
  {
    return {};
  }
}

}

//-----------------------------------------------------------------------------
class TrackFeaturesSprokitToolPrivate
{
public:
  image_io_sptr image_reader;
  convert_image_sptr image_converter;
  track_features_sptr feature_tracker;
};

QTE_IMPLEMENT_D_FUNC(TrackFeaturesSprokitTool)

//-----------------------------------------------------------------------------
TrackFeaturesSprokitTool::TrackFeaturesSprokitTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new TrackFeaturesSprokitToolPrivate)
{
  this->setText("&Track Features");
  this->setToolTip(
    "<nobr>Detect feature points in the images, compute feature descriptors, "
    "</nobr>and track the features across images.  Also run loop closure if "
    "configured to do so.");
}

//-----------------------------------------------------------------------------
TrackFeaturesSprokitTool::~TrackFeaturesSprokitTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs TrackFeaturesSprokitTool::outputs() const
{
  return Tracks | ActiveFrame;
}

//-----------------------------------------------------------------------------
bool TrackFeaturesSprokitTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasImagePaths())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires Images.");
    return false;
  }

  // Load configuration
  auto const config = readConfig("gui_track_features.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  if (!image_io::check_nested_algo_configuration(BLOCK_IR, config) ||
      !convert_image::check_nested_algo_configuration(BLOCK_CI, config))

  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  image_io::set_nested_algo_configuration(BLOCK_IR, config, d->image_reader);
  convert_image::set_nested_algo_configuration(BLOCK_CI, config, d->image_converter);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void TrackFeaturesSprokitTool::run()
{
  QTE_D();

  std::string pipe_file = "D:/export_controlled_data/telesculptor/sequence2/track_features_embedded.pipe";

  // Open pipeline description
  std::ifstream pipe_str;
  pipe_str.open(pipe_file, std::ifstream::in);
  if (!pipe_str)
  {
    return;
  }

  // create a embedded pipeline
  kwiver::embedded_pipeline ep;
  ep.build_pipeline(pipe_str);

  // Start pipeline and wait for it to finish
  ep.start();

  // Create dataset for input
  auto ds = kwiver::adapter::adapter_data_set::create();

  unsigned int frame = this->activeFrame();
  auto const& paths = this->imagePaths();

  unsigned int i = frame;
  for (; i < paths.size(); ++i)
  {
    auto const image = d->image_reader->load(paths[i]);
    auto const converted_image = d->image_converter->convert(image);
    // Set the metadata on the image.
    // For now, the only metadata is the filename of the image.
    auto md = std::make_shared<kwiver::vital::metadata>();
    md->add(NEW_METADATA_ITEM(kwiver::vital::VITAL_META_IMAGE_FILENAME, paths[i]));
    converted_image->set_metadata(md);
    auto timestamp = converted_image->get_metadata()->timestamp();

    ds->add_value("image", converted_image);
    ds->add_value("timestamp", timestamp);
    ep.send(ds);

    if (this->isCanceled())
    {
      break;
    }
    auto rds = ep.receive();
  }
  ep.send_end_of_input();

  ep.wait();

  bool end_reached = ep.at_end();
  auto rds = ep.receive();
  auto ix = rds->find("feature_track_set");

  auto d_ptr = ix->second->get_datum<kwiver::vital::feature_track_set_sptr>();

  this->updateTracks(d_ptr);


  //// Create dataset for input
  //auto ds = kwiver::adapter::adapter_data_set::create();

  //// Put OCV image in vital container
  //kwiver::vital::image_container_sptr img(new kwiver::arrows::ocv::image_container(cv_img));

  //ds->add_value("image", img);
  //ep.send(ds);
  //ep.send_end_of_input(); // indicate end of input

  //                        // Get results from pipeline
  //auto rds = ep.receive();
  //auto ix = rds->find("d_vector");
  //auto d_ptr = ix->second->get_datum<kwiver::vital::double_vector_sptr>();

  //ep.wait();



  //unsigned int frame = this->activeFrame();

  //auto tracks = this->tracks();

  //unsigned int i=frame;
  //for(; i<paths.size(); ++i)
  //{
  //  auto const image = d->image_reader->load(paths[i]);
  //  auto const converted_image = d->image_converter->convert(image);

  //  // Set the metadata on the image.
  //  // For now, the only metadata is the filename of the image.
  //  auto md = std::make_shared<kwiver::vital::metadata>();
  //  md->add( NEW_METADATA_ITEM( kwiver::vital::VITAL_META_IMAGE_FILENAME, paths[i] ) );
  //  converted_image->set_metadata(md);

  //  tracks = d->feature_tracker->track(tracks, i, converted_image);
  //  if (tracks)
  //  {
  //    tracks = kwiver::maptk::extract_feature_colors(tracks, *image, i);
  //  }

  //  // make a copy of the tool data
  //  auto data = std::make_shared<ToolData>();
  //  data->copyTracks(tracks);
  //  data->activeFrame = i;

  //  emit updated(data);
  //  if( this->isCanceled() )
  //  {
  //    break;
  //  }
  //}
  //this->updateTracks(tracks);
  //this->setActiveFrame(i);
}
