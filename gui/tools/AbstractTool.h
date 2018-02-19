/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
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

#ifndef MAPTK_ABSTRACTTOOL_H_
#define MAPTK_ABSTRACTTOOL_H_

#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>
#include <vital/types/landmark_map.h>
#include <vital/types/feature_track_set.h>
#include <vital/types/image_container.h>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>

#include <qtGlobal.h>

#include <QtGui/QAction>

class AbstractToolPrivate;

/// A class to hold data that is modified by the tool
class ToolData
{
public:
  typedef kwiver::vital::feature_track_set_sptr feature_track_set_sptr;
  typedef kwiver::vital::camera_map_sptr camera_map_sptr;
  typedef kwiver::vital::landmark_map_sptr landmark_map_sptr;
  typedef kwiver::vital::config_block_sptr config_block_sptr;
  typedef vtkSmartPointer<vtkImageData> depth_sptr;

  /// Deep copy the feature tracks into this data class
  void copyTracks(feature_track_set_sptr const&);

  /// Deep copy the cameras into this data class
  void copyCameras(camera_map_sptr const&);

  /// Deep copy the landmarks into this data class
  void copyLandmarks(landmark_map_sptr const&);

  /// Deep copy a depth image into this data class
  void copyDepth(depth_sptr const&);

  unsigned int activeFrame;
  std::string videoPath;
  feature_track_set_sptr tracks;
  depth_sptr active_depth;
  camera_map_sptr cameras;
  landmark_map_sptr landmarks;
  config_block_sptr config;
};

Q_DECLARE_METATYPE(std::shared_ptr<ToolData>)

class AbstractTool : public QAction
{
  Q_OBJECT

public:
  typedef kwiver::vital::feature_track_set_sptr feature_track_set_sptr;
  typedef kwiver::vital::camera_map_sptr camera_map_sptr;
  typedef kwiver::vital::landmark_map_sptr landmark_map_sptr;
  typedef kwiver::vital::config_block_sptr config_block_sptr;
  typedef vtkSmartPointer<vtkImageData> depth_sptr;

  enum Output
  {
    Tracks = 0x1,
    Cameras = 0x2,
    Landmarks = 0x4,
    ActiveFrame = 0x8,
    Depth = 0x10
  };
  Q_DECLARE_FLAGS(Outputs, Output)

  explicit AbstractTool(QObject* parent = 0);
  virtual ~AbstractTool();

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

  /// Set the image paths to be used as input to the tool.
  void setImagePaths(std::vector<std::string> const&);

  /// Set the feature tracks to be used as input to the tool.
  void setTracks(feature_track_set_sptr const&);

  /// Set the cameras to be used as input to the tool.
  void setCameras(camera_map_sptr const&);

  /// Set the landmarks to be used as input to the tool.
  void setLandmarks(landmark_map_sptr const&);

  /// Set the video source path.
  void setVideoPath(std::string const&);

  /// Set the config file if any
  void setConfig(config_block_sptr&);

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

  /// Get the active frame.
  unsigned int activeFrame() const;

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

signals:
  /// Emitted when the tool execution is completed.
  void completed();
  /// Emitted when an intermediate update of the data is available to show progress.
  void updated(std::shared_ptr<ToolData>);

  /// Emitted when the tool execution terminates due to user cancellation.
  void canceled();

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

private:
  QTE_DECLARE_PRIVATE_RPTR(AbstractTool)
  QTE_DECLARE_PRIVATE(AbstractTool)
  QTE_DISABLE_COPY(AbstractTool)
};

Q_DECLARE_OPERATORS_FOR_FLAGS(AbstractTool::Outputs)

#endif
