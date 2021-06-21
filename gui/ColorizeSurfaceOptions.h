// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_COLORIZESURFACEOPTIONS_H_
#define TELESCULPTOR_COLORIZESURFACEOPTIONS_H_

#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>

#include <qtGlobal.h>

#include <QWidget>

class vtkActor;

class ColorizeSurfaceOptionsPrivate;
class MeshColoration;

class ColorizeSurfaceOptions : public QWidget
{
  Q_OBJECT

public:
  explicit ColorizeSurfaceOptions(QString const& settingsGroup,
                                  QWidget* parent = nullptr,
                                  Qt::WindowFlags flags = {});
  ~ColorizeSurfaceOptions() override;

  void addColorDisplay(std::string name);

  void initFrameSampling(int nbFrames);
  int getFrameSampling() const;

  void setCurrentFrame(kwiver::vital::frame_id_t frame);
  void setOcclusionThreshold(double occlusionThreshold);
  void setRemoveOccluded(double removeOccluded);
  void setRemoveMasked(double removeMasked);
  double getOcclusionThreshold();
  void setActor(vtkActor* actor);
  void setVideoConfig(std::string const& path,
                      kwiver::vital::config_block_sptr config);
  kwiver::vital::config_block_sptr getVideoConfig() const;
  std::string getVideoPath() const;

  void setMaskConfig(std::string const& path,
                     kwiver::vital::config_block_sptr config);
  kwiver::vital::config_block_sptr getMaskConfig() const;
  std::string getMaskPath() const;

  void setCamera(kwiver::vital::frame_id_t id,
                 kwiver::vital::camera_sptr const& camera);
  kwiver::vital::camera_map_sptr getCameras() const;
  void enableMenu(bool);
  void forceColorize();

signals:
  void colorModeChanged(QString);

public slots:
  void changeColorDisplay();
  void colorize();
  void meshColorationHandleResult(MeshColoration* coloration);
  void enableAllFramesParameters(bool);
  void allFrameSelected();
  void currentFrameSelected();
  void updateOcclusionThreshold();
  void removeOccludedChanged(int removeOccluded);
  void removeMaskedChanged(int removeMasked);

private:

  QTE_DECLARE_PRIVATE_RPTR(ColorizeSurfaceOptions)
  QTE_DECLARE_PRIVATE(ColorizeSurfaceOptions)

  QTE_DISABLE_COPY(ColorizeSurfaceOptions)
};

#endif // COLORIZESURFACEOPTIONS_H
