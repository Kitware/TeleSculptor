/*ckwg +29
 * Copyright 2017-2018 by Kitware, Inc.
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

#ifndef TELESCULPTOR_ABSTRACTTOOL_H_
#define TELESCULPTOR_ABSTRACTTOOL_H_

#include <vital/config/config_block_types.h>
#include <vital/logger/logger.h>
#include <vital/types/camera_map.h>
#include <vital/types/landmark_map.h>
#include <vital/types/feature_track_set.h>
#include <vital/types/image_container.h>
#include <vital/types/sfm_constraints.h>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkBox.h>
#include <vtkNew.h>
#include <vtkStructuredGrid.h>

#include <qtGlobal.h>

#include <QAction>

class AbstractToolPrivate;

/// A class to hold data that is modified by the tool
class ToolData
{
public:
  typedef kwiver::vital::feature_track_set_sptr feature_track_set_sptr;
  typedef kwiver::vital::feature_track_set_changes_sptr feature_track_set_changes_sptr;
  typedef kwiver::vital::camera_map_sptr camera_map_sptr;
  typedef kwiver::vital::landmark_map_sptr landmark_map_sptr;
  typedef kwiver::vital::sfm_constraints_sptr sfm_constraints_sptr;
  typedef kwiver::vital::config_block_sptr config_block_sptr;
  typedef vtkSmartPointer<vtkImageData> depth_sptr;
  typedef std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string> > depth_lookup_sptr;
  typedef vtkSmartPointer<vtkImageData> fusion_sptr;

  /// Deep copy the feature tracks into this data class
  void copyTracks(feature_track_set_sptr const&);

  /// Deep copy of the feature track changes into this data class
  void copyTrackChanges(feature_track_set_changes_sptr const&);

  /// Deep copy the cameras into this data class
  void copyCameras(camera_map_sptr const&);

  /// Deep copy the landmarks into this data class
  void copyLandmarks(landmark_map_sptr const&);

  /// Deep copy a depth image into this data class
  void copyDepth(depth_sptr const&);

  /// Deep copy the list of depthmaps into this data class
  void copyDepthLookup(depth_lookup_sptr const&);

  /// Deep copy the fusion volume into this data class
  void copyFusion(fusion_sptr const& newVolume);

  /// Return true if the ToolData does not contain any large data updates
  bool isProgressOnly() const
  {
    return !tracks && !track_changes && !active_depth &&
           !cameras && !landmarks && !volume;
  }

  int maxFrame;
  unsigned int activeFrame;
  std::string videoPath;
  std::string maskPath;
  feature_track_set_sptr tracks;
  feature_track_set_changes_sptr track_changes;
  depth_sptr active_depth;
  camera_map_sptr cameras;
  landmark_map_sptr landmarks;
  sfm_constraints_sptr constraints;
  config_block_sptr config;
  kwiver::vital::logger_handle_t logger;
  int progress;
  std::string description;
  vtkNew<vtkBox> roi;
  depth_lookup_sptr depthLookup;
  fusion_sptr volume;
};

Q_DECLARE_METATYPE(std::shared_ptr<ToolData>)

class AbstractTool : public QAction
{
  Q_OBJECT

public:
  typedef kwiver::vital::feature_track_set_sptr feature_track_set_sptr;
  typedef kwiver::vital::feature_track_set_changes_sptr feature_track_set_changes_sptr;
  typedef kwiver::vital::camera_map_sptr camera_map_sptr;
  typedef kwiver::vital::landmark_map_sptr landmark_map_sptr;
  typedef kwiver::vital::sfm_constraints_sptr sfm_constraints_sptr;
  typedef kwiver::vital::config_block_sptr config_block_sptr;
  typedef vtkSmartPointer<vtkImageData> depth_sptr;
  typedef vtkSmartPointer<vtkImageData> fusion_sptr;

  enum Output
  {
    Tracks = 0x1,
    Cameras = 0x2,
    Landmarks = 0x4,
    ActiveFrame = 0x8,
    KeyFrames = 0x10,
    TrackChanges = 0x20,
    Depth = 0x40,
    Fusion = 0x80
  };
  Q_DECLARE_FLAGS(Outputs, Output)

  explicit AbstractTool(QObject* parent = 0);
  ~AbstractTool() override;

  /// Get the types of output produced by the tool.
  virtual Outputs outputs() const = 0;

  /// Get if the tool can be canceled.
  ///
  /// This method must be overridden by tool implementations. It should return
  /// \c false if the tool cannot be interrupted by the user. A return value of
  /// \c true implies that calling cancel() may have an effect.
  virtual bool isCancelable() const = 0;

  /// Return a shared pointer to the tools data
  std::shared_ptr<ToolData> data();

  /// Set the active frame to be used by the tool.
  void setActiveFrame(unsigned int frame);

  /// Set the frame number of the last frame.
  ///
  /// This sets the frame number of the last frame of the video. This is an
  /// optimization in order to allow some tools to correctly report their
  /// progress, since this value is expected to be known by the caller, but may
  /// be expensive for the tool to determine. This does \em not affect how many
  /// frames the tool will actually process.
  void setLastFrame(int);

  /// Set the feature tracks to be used as input to the tool.
  void setTracks(feature_track_set_sptr const&);

  /// Set the feature track changes to be used as input to the tool.
  void setTrackChanges(feature_track_set_changes_sptr const&);

  /// Set the cameras to be used as input to the tool.
  void setCameras(camera_map_sptr const&);

  /// Set the landmarks to be used as input to the tool.
  void setLandmarks(landmark_map_sptr const&);

  /// Set the sfm constraints to be used as input to the tool.
  void setSfmConstraints(sfm_constraints_sptr const&);

  /// Set the 3D region of interest
  void setROI(vtkBox *);

  /// Set the depth map lookup (frame to filename)
  void setDepthLookup(std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string> > const&);

  /// Set the video source path.
  void setVideoPath(std::string const&);

  /// Set the mask video source path.
  void setMaskPath(std::string const&);

  /// Set the config file if any
  void setConfig(config_block_sptr&);

  /// Set the tool data
  void setToolData(std::shared_ptr<ToolData>);

  /// Execute the tool.
  ///
  /// Tool implementations should override this method to verify that they have
  /// sufficient data before calling the base implementation.
  ///
  /// \param window Optional pointer to a widget to use as a context for any
  ///               dialogs that the tool may need to display.
  ///
  /// \return \c true if tool execution was started successfully, otherwise
  ///         \c false.
  virtual bool execute(QWidget* window = 0);

  /// Block until the tool has finished executing
  void wait();

  /// Get the active frame.
  unsigned int activeFrame() const;

  std::shared_ptr<std::map<kwiver::vital::frame_id_t, std::string> > depthLookup() const;
  vtkBox *ROI() const;

  /// Get tracks.
  ///
  /// This returns the new tracks resulting from the tool execution. If the
  /// tool does not output tracks, the tracks will be a copy of the input
  /// tracks.
  ///
  /// This may also be used by tool implementations to get the input tracks.
  /// (The tracks will be a copy that can be safely modified.)
  ///
  /// \warning Users must not call this method while the tool is executing,
  ///          as doing so may not be thread safe.
  feature_track_set_sptr tracks() const;

  /// Get cameras.
  ///
  /// This returns the new cameras resulting from the tool execution. If the
  /// tool does not output cameras, the cameras will be a copy of the input
  /// cameras.
  ///
  /// This may also be used by tool implementations to get the input cameras.
  /// (The cameras will be a copy that can be safely modified.)
  ///
  /// \warning Users must not call this method while the tool is executing,
  ///          as doing so may not be thread safe.
  camera_map_sptr cameras() const;

  /// Get landmarks.
  ///
  /// This returns the new landmarks resulting from the tool execution. If the
  /// tool does not output landmarks, the landmarks will be a copy of the input
  /// landmarks.
  ///
  /// This may also be used by tool implementations to get the input landmarks.
  /// (The landmarks will be a copy that can be safely modified.)
  ///
  /// \warning Users must not call this method while the tool is executing,
  ///          as doing so may not be thread safe.
  landmark_map_sptr landmarks() const;

  /// Get sfm constraints.
  ///
  /// This returns the SfM constraints used by the tool.
  ///
  /// This may also be used by tool implementations to get the input Sfm constraints.
  /// (The constraints will be a copy that can be safely modified.)
  ///
  /// \warning Users must not call this method while the tool is executing,
  ///          as doing so may not be thread safe.
  sfm_constraints_sptr sfmConstraints() const;

  /// Get tool progress.
  ///
  /// This returns the tool execution progress as an integer.
  int progress() const;

  /// Get tool execution description.
  ///
  /// This returns a textual description of what the tool is doing.
  QString description() const;

  /// Get track changes
  feature_track_set_changes_sptr track_changes() const;

  /// Get depth
  ToolData::depth_sptr depth() const;

  /// Get volume
  ToolData::fusion_sptr volume() const;

signals:
  /// Emitted when the tool execution is completed.
  void completed();
  /// Emitted when an intermediate update of the data is available to show progress.
  void updated(std::shared_ptr<ToolData>);

  /// Emitted when the tool execution terminates due to user cancellation.
  void canceled();

  /// Emitted when the tool execution wishes to save a result to disk
  void saved(std::shared_ptr<ToolData>);

  /// Emitted when the tool execution terminates due to an execution error.
  void failed(QString reason);

public slots:
  /// Ask the tool to cancel execution.
  ///
  /// This sets a flag indicating that the user has requested the tool
  /// execution should halt. The tool may or may not honor such a request.
  ///
  /// \sa canceled, isCancelable
  virtual void cancel();

protected:
  /// Execute the tool.
  ///
  /// This method must be overridden by tool implementations. The default
  /// implementation of execute() calls this method in a separate thread.
  virtual void run() = 0;

  /// Check if the user has requested that tool execution be canceled.
  bool isCanceled() const;

  /// Test if the tool has track data.
  ///
  /// \return \c true if the tool data has a non-zero number of feature tracks,
  ///         otherwise \c false
  bool hasTracks() const;

  /// Test if the tool has camera data.
  ///
  /// \return \c true if the tool data has a non-zero number of cameras,
  ///         otherwise \c false
  bool hasCameras() const;

  /// Test if the tool has landmark data.
  ///
  /// \return \c true if the tool data has a non-zero number of landmarks,
  ///         otherwise \c false
  bool hasLandmarks() const;

  /// Test if the tool has depth maps
  ///
  /// \return \c true if the tool data has a non-zero number of depthMaps,
  ///         otherwise \c false
  bool hasDepthLookup() const;

  /// Test if the tool has video data.
  ///
  /// \return \c true if the tool data has an associated video source,
  ///         otherwise \c false
  bool hasVideoSource() const;

  /// Set the tracks produced by the tool.
  ///
  /// This sets the tracks that are produced by the tool as output. Unlike
  /// setTracks, this does not make a deep copy of the provided tracks.
  void updateTracks(feature_track_set_sptr const&);

  /// Set the cameras produced by the tool.
  ///
  /// This sets the cameras that are produced by the tool as output. Unlike
  /// setCameras, this does not make a deep copy of the provided cameras.
  void updateCameras(camera_map_sptr const&);

  /// Set the depth map produced by the tool.
  void updateDepth(depth_sptr const& newDepth);

  /// Set the landmarks produced by the tool.
  ///
  /// This sets the landmarks that are produced by the tool as output. Unlike
  /// setCameras, this does not make a deep copy of the provided landmarks.
  void updateLandmarks(landmark_map_sptr const&);

  /// Set tool progress.
  ///
  /// This returns the tool execution progress as an integer.
  void updateProgress(int value, int maximum = 100);

  /// Set the volume produced by the tool
  void updateFusion(vtkSmartPointer<vtkImageData>);

  /// Set tool execution description.
  ///
  /// This returns a textual description of what the tool is doing.
  void setDescription(const QString& desc);

private:
  QTE_DECLARE_PRIVATE_RPTR(AbstractTool)
  QTE_DECLARE_PRIVATE(AbstractTool)
  QTE_DISABLE_COPY(AbstractTool)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(AbstractTool::Outputs)

#endif
