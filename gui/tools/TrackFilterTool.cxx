// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "TrackFilterTool.h"
#include "GuiCommon.h"

#include <vital/algo/algorithm.txx>
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
  if (!kwiver::vital::check_nested_algo_configuration<filter_tracks>(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  kwiver::vital::set_nested_algo_configuration<filter_tracks>(BLOCK, config, d->algorithm);

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
