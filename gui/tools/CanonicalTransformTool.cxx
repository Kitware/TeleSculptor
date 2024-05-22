// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "CanonicalTransformTool.h"
#include "GuiCommon.h"

#include <vital/algo/algorithm.txx>
#include <vital/algo/estimate_canonical_transform.h>

#include <arrows/mvg/transform.h>

#include <QMessageBox>

using kwiver::vital::algo::estimate_canonical_transform;
using kwiver::vital::algo::estimate_canonical_transform_sptr;

namespace
{
static char const* const BLOCK = "can_tfm_estimator";
}

//-----------------------------------------------------------------------------
class CanonicalTransformToolPrivate
{
public:
  estimate_canonical_transform_sptr algorithm;
};

QTE_IMPLEMENT_D_FUNC(CanonicalTransformTool)

//-----------------------------------------------------------------------------
CanonicalTransformTool::CanonicalTransformTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new CanonicalTransformToolPrivate)
{
  this->setText("&Align");
  this->setToolTip(
    "<nobr>Compute and apply a world transform so that the landmarks are "
    "centered</nobr> about the origin, scaled to unit variance, and oriented "
    "to best align with a <tt>z=0</tt> ground plane, with the cameras above "
    "the same in the <tt>+Z</tt> direction");
}

//-----------------------------------------------------------------------------
CanonicalTransformTool::~CanonicalTransformTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs CanonicalTransformTool::outputs() const
{
  return Cameras | Landmarks;
}

//-----------------------------------------------------------------------------
bool CanonicalTransformTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasLandmarks())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires landmarks.");
    return false;
  }

  // Merge project config with default config file
  auto const config = readConfig("gui_align.conf");

  // Check configuration
  if (!config)
  {
    QMessageBox::critical(
      window, "Configuration error",
      "No configuration data was found. Please check your installation.");
    return false;
  }

  config->merge_config(this->data()->config);
  if (!kwiver::vital::check_nested_algo_configuration<estimate_canonical_transform>(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  kwiver::vital::set_nested_algo_configuration<estimate_canonical_transform>(
    BLOCK, config, d->algorithm);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void CanonicalTransformTool::run()
{
  QTE_D();

  auto const& xf =
    d->algorithm->estimate_transform(this->cameras(), this->landmarks());

  this->updateCameras(kwiver::arrows::mvg::transform(this->cameras(), xf));
  this->updateLandmarks(kwiver::arrows::mvg::transform(this->landmarks(), xf));
}
