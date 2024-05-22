// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "InitCamerasLandmarksTool.h"
#include "GuiCommon.h"

#include <vital/algo/algorithm.txx>
#include <vital/algo/initialize_cameras_landmarks.h>

#include <QMessageBox>

using kwiver::vital::algo::initialize_cameras_landmarks;
using kwiver::vital::algo::initialize_cameras_landmarks_sptr;

namespace
{
static char const* const BLOCK = "initializer";
static char const* const CONFIG_FILE = "gui_initialize.conf";
}

//-----------------------------------------------------------------------------
class InitCamerasLandmarksToolPrivate
{
public:
  initialize_cameras_landmarks_sptr algorithm;
  size_t num_frames;
};

QTE_IMPLEMENT_D_FUNC(InitCamerasLandmarksTool)

//-----------------------------------------------------------------------------
InitCamerasLandmarksTool::InitCamerasLandmarksTool(QObject* parent)
  : AbstractTool(parent), d_ptr(new InitCamerasLandmarksToolPrivate)
{
  this->setText("&Estimate Cameras/Landmarks");
  this->setToolTip(
    "<nobr>Estimate cameras and landmarks from a set of feature tracks</nobr>");
}

//-----------------------------------------------------------------------------
InitCamerasLandmarksTool::~InitCamerasLandmarksTool()
{
}

//-----------------------------------------------------------------------------
AbstractTool::Outputs InitCamerasLandmarksTool::outputs() const
{
  return Cameras | Landmarks | TrackChanges | Tracks;
}

//-----------------------------------------------------------------------------
bool InitCamerasLandmarksTool::execute(QWidget* window)
{
  QTE_D();

  // Check inputs
  if (!this->hasTracks())
  {
    QMessageBox::information(
      window, "Insufficient data",
      "This operation requires feature tracks.");
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
  if (!kwiver::vital::check_nested_algo_configuration<initialize_cameras_landmarks>(BLOCK, config))
  {
    QMessageBox::critical(
      window, "Configuration error",
      "An error was found in the algorithm configuration.");
    return false;
  }

  if (config->has_value("variable_lens"))
  {
    bool variable_lens = config->get_value<bool>("variable_lens");
    config->set_value("initializer:keyframe:force_common_intrinsics",
                      !variable_lens);
  }

  // Create algorithm from configuration
  kwiver::vital::set_nested_algo_configuration<initialize_cameras_landmarks>(
    BLOCK, config, d->algorithm);

  // Set the callback to receive updates
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  typedef initialize_cameras_landmarks::callback_t callback_t;
  callback_t cb = std::bind(&InitCamerasLandmarksTool::callback_handler, this, _1, _2, _3);
  d->algorithm->set_callback(cb);

  // Hand off to base class
  return AbstractTool::execute(window);
}

//-----------------------------------------------------------------------------
void InitCamerasLandmarksTool::run()
{
  QTE_D();

  auto cp = this->cameras();
  auto lp = this->landmarks();
  auto tp = this->tracks();
  auto sp = this->sfmConstraints();

  // If cp is Null the initialize algorithm will create all cameras.
  // If not Null it will only create cameras if they are in the map but Null.
  // So we need to add placeholders for missing cameras to the map
  if (cp)
  {
    using kwiver::vital::frame_id_t;
    using kwiver::vital::camera_map;
    std::set<frame_id_t> frame_ids = tp->all_frame_ids();
    d->num_frames = frame_ids.size();
    camera_map::map_camera_t all_cams = cp->cameras();

    for (auto const& id : frame_ids)
    {
      if (all_cams.find(id) == all_cams.end())
      {
        all_cams[id] = kwiver::vital::camera_sptr();
      }
    }
    cp = std::make_shared<kwiver::vital::simple_camera_map>(all_cams);
  }

  // If lp is Null the initialize algorithm will create all landmarks.
  // If not Null it will only create landmarks if they are in the map but Null.
  // So we need to add placeholders for missing landmarks to the map
  if (lp)
  {
    using kwiver::vital::track_id_t;
    using kwiver::vital::landmark_map;
    std::set<track_id_t> track_ids = tp->all_track_ids();
    landmark_map::map_landmark_t all_lms = lp->landmarks();

    for (auto const& id : track_ids)
    {
      if (all_lms.find(id) == all_lms.end())
      {
        all_lms[id] = kwiver::vital::landmark_sptr();
      }
    }
    lp = std::make_shared<kwiver::vital::simple_landmark_map>(all_lms);
  }

  this->setDescription("Initializing Cameras and Landmarks");
  this->updateProgress(0, 100);
  auto data = std::make_shared<ToolData>();
  emit updated(data);

  d->algorithm->initialize(cp, lp, tp, sp);

  this->updateCameras(cp);
  this->updateLandmarks(lp);
  this->updateTracks(tp);
}

//-----------------------------------------------------------------------------
bool InitCamerasLandmarksTool::callback_handler(camera_map_sptr cameras,
                                                landmark_map_sptr landmarks,
                                                feature_track_set_changes_sptr track_changes)
{
  QTE_D();
  if (cameras || landmarks || track_changes)
  {
    unsigned int percent_complete =
      static_cast<unsigned int>(cameras->size() * 100 / d->num_frames);
    // Create overall status message
    std::stringstream ss;
    ss << "Initialized " << cameras->size() << " of " << d->num_frames << " cameras";
    this->setDescription(QString::fromStdString(ss.str()));
    this->updateProgress(percent_complete);

    // make a copy of the tool data
    auto data = std::make_shared<ToolData>();
    data->copyCameras(cameras);
    data->copyLandmarks(landmarks);
    data->copyTrackChanges(track_changes);
    data->description = description().toStdString();
    data->progress = progress();
    data->activeFrame = cameras->cameras().rbegin()->first;
    emit updated(data);
  }
  return !this->isCanceled();
}
