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

#include "TrackFilterTool.h"
#include "GuiCommon.h"

#include <vital/algo/filter_tracks.h>

#include <QMessageBox>

using kwiver::vital::algo::filter_tracks;
using kwiver::vital::algo::filter_tracks_sptr;

namespace
{
static char const* const BLOCK = "track_filter";
}

//-----------------------------------------------------------------------------
class TrackFilterToolPrivate
{
public:
  filter_tracks_sptr algorithm;
};

QTE_IMPLEMENT_D_FUNC(TrackFilterTool)

//-----------------------------------------------------------------------------
TrackFilterTool::TrackFilterTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new TrackFilterToolPrivate)
{
  this->setText("&Filter Tracks");
  this->setToolTip(
    "<nobr>Filter feature tracks to remove short and redundant tracks to speed "
    "</nobr>up bundle adjustment");
}

//-----------------------------------------------------------------------------
TrackFilterTool::~TrackFilterTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs TrackFilterTool::outputs() const
{
  return Tracks;
}

//-----------------------------------------------------------------------------
bool TrackFilterTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasTracks())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires tracks.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_filter_tracks.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!filter_tracks::check_nested_algo_configuration(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  filter_tracks::set_nested_algo_configuration(BLOCK, config, d->algorithm);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void TrackFilterTool::run()
{
  QTE_D();

  auto tp = d->algorithm->filter(this->tracks());
  auto ftp = std::static_pointer_cast<kwiver::vital::feature_track_set>(tp);

  this->updateTracks(ftp);
}
