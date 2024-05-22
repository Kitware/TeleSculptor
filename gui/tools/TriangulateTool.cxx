// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "TriangulateTool.h"
#include "GuiCommon.h"

#include <vital/algo/algorithm.txx>
#include <vital/algo/triangulate_landmarks.h>

#include <QMessageBox>

using kwiver::vital::algo::triangulate_landmarks;
using kwiver::vital::algo::triangulate_landmarks_sptr;

namespace
{
static char const* const BLOCK = "triangulator";
static char const* const CONFIG_FILE = "gui_triangulate.conf";
}

//-----------------------------------------------------------------------------
class TriangulateToolPrivate
{
public:
  triangulate_landmarks_sptr algorithm;
};

QTE_IMPLEMENT_D_FUNC(TriangulateTool)

//-----------------------------------------------------------------------------
TriangulateTool::TriangulateTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new TriangulateToolPrivate)
{
  this->data()->logger =
    kwiver::vital::get_logger("telesculptor.tools.triangulate");

  this->setText("&Triangulate Landmarks");
  this->setToolTip(
    "<nobr>Triangulate landmarks in current set of cameras.</nobr>");
}

//-----------------------------------------------------------------------------
TriangulateTool::~TriangulateTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs TriangulateTool::outputs() const
{
  return Landmarks | Tracks;
}

//-----------------------------------------------------------------------------
bool TriangulateTool::execute(QWidget* window)
{
  QTE_D();

  // Check inputs
  if (!this->hasTracks() || !this->hasCameras())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires feature tracks and cameras.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig(CONFIG_FILE);

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      QString("No configuration data was found. Looking for \"")
      + CONFIG_FILE + "\". Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!kwiver::vital::check_nested_algo_configuration<triangulate_landmarks>(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  kwiver::vital::set_nested_algo_configuration<triangulate_landmarks>(
    BLOCK, config, d->algorithm);

  // Hand off to base class
  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void TriangulateTool::run()
{
  QTE_D();

  auto cp = this->cameras();
  auto tp = this->tracks();

  kwiver::vital::landmark_map::map_landmark_t init_lms;
  std::set<kwiver::vital::track_id_t> track_ids = tp->all_track_ids();

  // Landmarks to triangulate must be created in the map.
  // These could be initialized to Null, but some versions of KWIVER may
  // crash, so it is safer to give them a value
  kwiver::vital::vector_3d init_loc(0, 0, 0);
  for (auto const& id : track_ids)
  {
    init_lms[id] = std::make_shared<kwiver::vital::landmark_d>(init_loc);
  }
  kwiver::vital::landmark_map_sptr lp =
    std::make_shared<kwiver::vital::simple_landmark_map>(init_lms);

  d->algorithm->triangulate(cp, tp, lp);

  LOG_INFO(this->data()->logger, "Triangulated " << lp->size()
           << " out of " << init_lms.size() << " tracks.");

  this->updateLandmarks(lp);
  this->updateTracks(tp);
}

