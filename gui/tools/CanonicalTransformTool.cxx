/*ckwg +29
 * Copyright 2016-2019 by Kitware, Inc.
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

#include "CanonicalTransformTool.h"
#include "GuiCommon.h"

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
  if (!estimate_canonical_transform::check_nested_algo_configuration(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  // Create algorithm from configuration
  estimate_canonical_transform::set_nested_algo_configuration(
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
