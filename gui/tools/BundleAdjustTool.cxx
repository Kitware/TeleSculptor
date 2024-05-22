// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "BundleAdjustTool.h"
#include "GuiCommon.h"

#include <vital/algo/algorithm.txx>
#include <vital/algo/bundle_adjust.h>

#include <QMessageBox>

using kwiver::vital::algo::bundle_adjust;
using kwiver::vital::algo::bundle_adjust_sptr;

namespace
{
static char const* const BLOCK = "bundle_adjuster";
}

//-----------------------------------------------------------------------------
class BundleAdjustToolPrivate
{
public:
  bundle_adjust_sptr algorithm;
};

QTE_IMPLEMENT_D_FUNC(BundleAdjustTool)

//-----------------------------------------------------------------------------
BundleAdjustTool::BundleAdjustTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new BundleAdjustToolPrivate)
{
  this->setText("&Refine Solution");
  this->setToolTip(
    "<nobr>Apply bundle adjustment to the cameras and landmarks in order to"
    "</nobr> refine the quality of the 3D reconstruction");
}

//-----------------------------------------------------------------------------
BundleAdjustTool::~BundleAdjustTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs BundleAdjustTool::outputs() const
{
  return Cameras | Landmarks;
}

//-----------------------------------------------------------------------------
bool BundleAdjustTool::execute(QWidget* window)
{
  QTE_D();

  // Check inputs
  if (!this->hasLandmarks() || !this->hasCameras() || !this->hasTracks())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires feature tracks, cameras and landmarks.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_bundle_adjust.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!kwiver::vital::check_nested_algo_configuration<bundle_adjust>(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  kwiver::vital::set_nested_algo_configuration<bundle_adjust>(BLOCK, config, d->algorithm);

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  typedef bundle_adjust::callback_t callback_t;
  callback_t cb = std::bind(&BundleAdjustTool::callback_handler, this, _1, _2);
  d->algorithm->set_callback(cb);

  // Hand off to base class
  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void BundleAdjustTool::run()
{
  QTE_D();

  auto cp = this->cameras();
  auto lp = this->landmarks();
  auto tp = this->tracks();
  auto sp = this->sfmConstraints();

  d->algorithm->optimize(cp, lp, tp, sp);

  this->updateCameras(cp);
  this->updateLandmarks(lp);
}

//-----------------------------------------------------------------------------
bool BundleAdjustTool::callback_handler(camera_map_sptr cameras,
                                        landmark_map_sptr landmarks)
{
  if (cameras || landmarks)
  {
    // make a copy of the tool data
    auto data = std::make_shared<ToolData>();
    data->copyCameras(cameras);
    data->copyLandmarks(landmarks);
    emit updated(data);
  }

  return !this->isCanceled();
}
