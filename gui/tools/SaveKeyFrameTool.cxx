/*ckwg +29
 * Copyright 2016 by Kitware, Inc.
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

#include "SaveKeyFrameTool.h"

#include <vital/algo/video_input.h>

#include <QtCore/QDebug>
#include <QtGui/QMessageBox>

using kwiver::vital::algo::video_input;
using kwiver::vital::algo::video_input_sptr;

namespace
{
static char const* const BLOCK_VR = "video_reader";
}

QTE_IMPLEMENT_D_FUNC(SaveKeyFrameTool)

//-----------------------------------------------------------------------------
class SaveKeyFrameToolPrivate
{
public:
  video_input_sptr video_reader;
};

//-----------------------------------------------------------------------------
SaveKeyFrameTool::SaveKeyFrameTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new SaveKeyFrameToolPrivate)
{
  this->setText("Save Key Frames");
  this->setToolTip(
    "Saves the key frames from the Track Features to disk.");
}

//-----------------------------------------------------------------------------
SaveKeyFrameTool::~SaveKeyFrameTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs SaveKeyFrameTool::outputs() const
{
  return KeyFrames;
}

//-----------------------------------------------------------------------------
bool SaveKeyFrameTool::execute(QWidget* window)
{
  QTE_D();

  if (!this->hasTracks() || !this->hasVideoSource())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires tracks and a video source.");
    return false;
  }

  if (!video_input::check_nested_algo_configuration(
    BLOCK_VR, this->data()->config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  video_input::set_nested_algo_configuration(
    BLOCK_VR, this->data()->config, d->video_reader);

  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void SaveKeyFrameTool::run()
{
  QTE_D();

  kwiver::vital::timestamp currentTimestamp;

  d->video_reader->open(this->data()->videoPath);

  // Uncomment once keyframes issues in kwiver are resolved
  // for (auto const& frame: this->tracks()->keyframes())
  std::set<kwiver::vital::frame_id_t> testKeyFrames = {1, 301, 401, 581, 1001};
  for (auto const& frame: testKeyFrames)
  {
    if (d->video_reader->seek_frame(currentTimestamp, frame))
    {
      qWarning() << "FRAME: " << frame;
    }
    else
    {
      qWarning() << "Key frame " << frame << " not available in video source.";
    }
  }

  d->video_reader->close();
}
