// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#include "AbstractTool.h"

#include <qtStlUtil.h>

#include <QThread>

#include <atomic>

//-----------------------------------------------------------------------------
class AbstractToolPrivate : public QThread
{
public:
  AbstractToolPrivate(AbstractTool* q)
    : data(std::make_shared<ToolData>()), q_ptr(q) {}

  virtual void run() override;

  std::shared_ptr<ToolData> data;

  std::atomic<bool> cancelRequested;

protected:
  QTE_DECLARE_PUBLIC_PTR(AbstractTool)
  QTE_DECLARE_PUBLIC(AbstractTool)
};

QTE_IMPLEMENT_D_FUNC(AbstractTool)

//-----------------------------------------------------------------------------
void AbstractToolPrivate::run()
{
  QTE_Q();

  try
  {
    q->run();
  }
  catch (const std::exception& e)
  {
    emit q->failed(QString::fromLocal8Bit(e.what()));
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyTracks(feature_track_set_sptr const& newTracks)
{
  if (newTracks)
  {
    this->tracks = std::dynamic_pointer_cast<kwiver::vital::feature_track_set>(
      newTracks->clone());
  }
  else
  {
    this->tracks = feature_track_set_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyTrackChanges(feature_track_set_changes_sptr const& newTrackChanges)
{
  if (newTrackChanges)
  {
    this->track_changes = std::dynamic_pointer_cast<kwiver::vital::feature_track_set_changes>(
      newTrackChanges->clone());
  }
  else
  {
    this->track_changes = feature_track_set_changes_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyCameras(camera_map_sptr const& newCameras)
{
  if (newCameras)
  {
    auto copiedCameras = kwiver::vital::camera_map::map_camera_t{};
    foreach (auto const& ci, newCameras->cameras())
    {
      copiedCameras.emplace(ci.first, ci.second->clone());
    }
    this->cameras =
      std::make_shared<kwiver::vital::simple_camera_map>(copiedCameras);
  }
  else
  {
    this->cameras = camera_map_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyLandmarks(landmark_map_sptr const& newLandmarks)
{
  if (newLandmarks)
  {
    auto copiedLandmarks = kwiver::vital::landmark_map::map_landmark_t{};
    foreach (auto const& ci, newLandmarks->landmarks())
    {
      copiedLandmarks.emplace(ci.first, ci.second->clone());
    }
    this->landmarks =
      std::make_shared<kwiver::vital::simple_landmark_map>(copiedLandmarks);
  }
  else
  {
    this->landmarks = landmark_map_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyDepth(depth_sptr const& newDepth)
{
  this->active_depth = vtkSmartPointer<vtkImageData>::New();
  if (newDepth)
  {
    this->active_depth->DeepCopy(newDepth);
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyDepthLookup(depth_lookup_sptr const& newDepthLookup)
{
  if (newDepthLookup)
  {
    depth_lookup_sptr depths(new std::map<kwiver::vital::frame_id_t, std::string>());
    *depths = *newDepthLookup;
    this->depthLookup = depths;
  }
  else
  {
    this->cameras = camera_map_sptr();
  }
}

//-----------------------------------------------------------------------------
void ToolData::copyFusion(fusion_sptr const& newVolume)
{
  this->volume = vtkSmartPointer<vtkImageData>::New();
  if (newVolume)
  {
    this->volume->DeepCopy(newVolume);
  }
}

//-----------------------------------------------------------------------------
AbstractTool::AbstractTool(QObject* parent)
  : QAction(parent), d_ptr(new AbstractToolPrivate(this))
{
  QTE_D();
  connect(d, &QThread::finished, this, &AbstractTool::completed);
}

//-----------------------------------------------------------------------------
AbstractTool::~AbstractTool()
{
  QTE_D();
  d->wait();
}

//-----------------------------------------------------------------------------
std::shared_ptr<ToolData> AbstractTool::data()
{
  QTE_D();
  return d->data;
}

//-----------------------------------------------------------------------------
kwiver::vital::frame_id_t AbstractTool::activeFrame() const
{
  QTE_D();
  return d->data->activeFrame;
}

//-----------------------------------------------------------------------------
std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string>> AbstractTool::depthLookup() const
{
  QTE_D();
  return d->data->depthLookup;
}

//-----------------------------------------------------------------------------
vtkBox* AbstractTool::ROI() const
{
  QTE_D();
  return d->data->roi.Get();
}

//-----------------------------------------------------------------------------
kwiver::vital::feature_track_set_sptr AbstractTool::tracks() const
{
  QTE_D();
  return d->data->tracks;
}

//-----------------------------------------------------------------------------
kwiver::vital::camera_map_sptr AbstractTool::cameras() const
{
  QTE_D();
  return d->data->cameras;
}

//-----------------------------------------------------------------------------
AbstractTool::landmark_map_sptr AbstractTool::landmarks() const
{
  QTE_D();
  return d->data->landmarks;
}

//-----------------------------------------------------------------------------
AbstractTool::sfm_constraints_sptr AbstractTool::sfmConstraints() const
{
  QTE_D();
  return d->data->constraints;
}

//-----------------------------------------------------------------------------
AbstractTool::feature_track_set_changes_sptr AbstractTool::track_changes() const
{
  QTE_D();
  return d->data->track_changes;
}

//-----------------------------------------------------------------------------
AbstractTool::depth_sptr AbstractTool::depth() const
{
  QTE_D();
  return d->data->active_depth;
}

//-----------------------------------------------------------------------------
AbstractTool::fusion_sptr AbstractTool::volume() const
{
  QTE_D();
  return d->data->volume;
}

//-----------------------------------------------------------------------------
  void AbstractTool::cancel()
{
  QTE_D();
  d->cancelRequested = true;
}

//-----------------------------------------------------------------------------
void AbstractTool::setActiveFrame(kwiver::vital::frame_id_t frame)
{
  QTE_D();
  d->data->activeFrame = frame;
}

//-----------------------------------------------------------------------------
void AbstractTool::setLastFrame(kwiver::vital::frame_id_t count)
{
  QTE_D();
  d->data->maxFrame = count;
}

//-----------------------------------------------------------------------------
void AbstractTool::setTracks(feature_track_set_sptr const& newTracks)
{
  QTE_D();
  d->data->copyTracks(newTracks);
}

//-----------------------------------------------------------------------------
void AbstractTool::setTrackChanges(feature_track_set_changes_sptr const& newTrackChanges)
{
  QTE_D();
  d->data->copyTrackChanges(newTrackChanges);
}

//-----------------------------------------------------------------------------
void AbstractTool::setCameras(camera_map_sptr const& newCameras)
{
  QTE_D();
  d->data->copyCameras(newCameras);
}

//-----------------------------------------------------------------------------
void AbstractTool::setLandmarks(landmark_map_sptr const& newLandmarks)
{
  QTE_D();
  d->data->copyLandmarks(newLandmarks);
}

//-----------------------------------------------------------------------------
void AbstractTool::setSfmConstraints(sfm_constraints_sptr const& newConstraints)
{
  QTE_D();
  if (newConstraints)
  {
    d->data->constraints = std::make_shared<kwiver::vital::sfm_constraints>(*newConstraints);
  }
  else
  {
    d->data->constraints = std::make_shared<kwiver::vital::sfm_constraints>();
  }
}

//-----------------------------------------------------------------------------
void AbstractTool::setROI(vtkBox* newROI)
{
  QTE_D();
  d->data->roi->SetBounds(newROI->GetBounds());
}

//-----------------------------------------------------------------------------
void AbstractTool::setDepthLookup(std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string>> const& newDepthLookup)
{
  QTE_D();
  d->data->copyDepthLookup(newDepthLookup);
}

//-----------------------------------------------------------------------------
void AbstractTool::setVideoPath(std::string const& path)
{
  QTE_D();
  d->data->videoPath = path;
}

//-----------------------------------------------------------------------------
void AbstractTool::setMaskPath(std::string const& path)
{
  QTE_D();
  d->data->maskPath = path;
}

//-----------------------------------------------------------------------------
void AbstractTool::setConfig(config_block_sptr& config)
{
  QTE_D();
  d->data->config = config;
}

//-----------------------------------------------------------------------------
void AbstractTool::setToolData(std::shared_ptr<ToolData> data)
{
  QTE_D();
  d->data = data;
}

//-----------------------------------------------------------------------------
bool AbstractTool::execute(QWidget* window)
{
  QTE_D();

  // Reset progress reporting parameters
  this->updateProgress(0);
  this->setDescription("");
  d->cancelRequested = false;
  d->start();
  return true;
}

//-----------------------------------------------------------------------------
void AbstractTool::wait()
{
  QTE_D();
  d->wait();
}

//-----------------------------------------------------------------------------
bool AbstractTool::isCanceled() const
{
  QTE_D();
  return d->cancelRequested;
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasTracks() const
{
  QTE_D();
  return d->data->tracks && d->data->tracks->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasCameras() const
{
  QTE_D();
  return d->data->cameras && d->data->cameras->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasLandmarks() const
{
  QTE_D();
  return d->data->landmarks && d->data->landmarks->size();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasDepthLookup() const
{
  QTE_D();
  return !d->data->depthLookup->empty();
}

//-----------------------------------------------------------------------------
bool AbstractTool::hasVideoSource() const
{
  QTE_D();
  if (d->data->videoPath == "" || !d->data->config)
  {
    return false;
  }
  else
  {
    return true;
  }
}

//-----------------------------------------------------------------------------
void AbstractTool::updateTracks(feature_track_set_sptr const& newTracks)
{
  QTE_D();
  d->data->tracks = newTracks;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateDepth(depth_sptr const& newDepth)
{
  QTE_D();
  d->data->active_depth = newDepth;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateCameras(camera_map_sptr const& newCameras)
{
  QTE_D();
  d->data->cameras = newCameras;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateLandmarks(landmark_map_sptr const& newLandmarks)
{
  QTE_D();
  d->data->landmarks = newLandmarks;
}

//-----------------------------------------------------------------------------
int AbstractTool::progress() const
{
  QTE_D();
  int p = d->data->progress;
  p = p < 0 ? 0 : p;
  p = p > 100 ? 100 : p;
  return p;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateProgress(int value, int maximum)
{
  QTE_D();
  // FIXME also report progress range rather than forcing to 0-100
  d->data->progress = (100 * value) / maximum;
}

//-----------------------------------------------------------------------------
void AbstractTool::updateFusion(vtkSmartPointer<vtkImageData> newVolume)
{
  QTE_D();
  d->data->volume = newVolume;
}

//-----------------------------------------------------------------------------
QString AbstractTool::description() const
{
  QTE_D();
  return qtString(d->data->description);
}

//-----------------------------------------------------------------------------
void AbstractTool::setDescription(QString const& desc)
{
  QTE_D();
  d->data->description = stdString(desc);
}
