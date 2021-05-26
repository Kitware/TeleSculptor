// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "NeckerReversalTool.h"

#include <arrows/mvg/necker_reverse.h>

#include <QMessageBox>

//-----------------------------------------------------------------------------
NeckerReversalTool::NeckerReversalTool(QObject* parent)
  : AbstractTool(parent)
{
  this->setText("Reverse (&Necker)");
  this->setToolTip(
    "<nobr>Attempt to correct a degenerate refinement (which can occur due "
    "to</nobr> the Necker cube phenomena) by computing a best fit plane to "
    "the landmarks, mirroring the landmarks about said plane, and rotating "
    "the cameras 180&deg; about their respective optical axes and 180&deg; "
    "about the best fit plane normal where each camera's optical axis "
    "intersects said plane.");
}

//-----------------------------------------------------------------------------
NeckerReversalTool::~NeckerReversalTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs NeckerReversalTool::outputs() const
{
  return Cameras | Landmarks;
}

//-----------------------------------------------------------------------------
bool NeckerReversalTool::execute(QWidget* window)
{
  if (!this->hasLandmarks())
  {
    QMessageBox::information(
      window, "Insufficient data", "This operation requires landmarks.");
    return false;
  }

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void NeckerReversalTool::run()
{
  auto cp = this->cameras();
  auto lp = this->landmarks();

  kwiver::arrows::mvg::necker_reverse(cp, lp);

  this->updateCameras(cp);
  this->updateLandmarks(lp);
}
