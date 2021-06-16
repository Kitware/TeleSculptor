// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_VOLUMEOPTIONS_H_
#define TELESCULPTOR_VOLUMEOPTIONS_H_

#include <vital/config/config_block_types.h>
#include <vital/types/camera_map.h>

#include <qtGlobal.h>

#include <QWidget>

class vtkPolyData;
class vtkActor;
class vtkDataArray;

class VolumeOptionsPrivate;

class VolumeOptions : public QWidget
{
  Q_OBJECT

public:
  explicit VolumeOptions(QString const& settingsGroup,
                         QWidget* parent = nullptr,
                         Qt::WindowFlags flags = {});
  ~VolumeOptions() override;

  void setActor(vtkActor* actor);

  void initFrameSampling(int nbFrames);
  int getFrameSampling() const;
  double getOcclusionThreshold() const;

  void setCamera(kwiver::vital::frame_id_t id,
                 kwiver::vital::camera_sptr const& camera);
  kwiver::vital::camera_map_sptr getCameras() const;

  void setVideoConfig(std::string const& path,
                      kwiver::vital::config_block_sptr config);
  kwiver::vital::config_block_sptr getVideoConfig() const;
  std::string getVideoPath() const;
  void setMaskConfig(std::string const& path,
                     kwiver::vital::config_block_sptr config);
  kwiver::vital::config_block_sptr getMaskConfig() const;
  std::string getMaskPath() const;
  void colorize();
  void forceColorize();

  void setCurrentFrame(kwiver::vital::frame_id_t);

  enum SurfaceColor
  {
    NO_COLOR,
    IMAGE_COLOR,
    ORIGINAL_COLOR
  };
  void setColorizeSurface(int surfaceColor, bool blockSignals = false);
  void setOriginalColorArray(vtkDataArray* dataArray);
  bool isColorOptionsEnabled();

  static bool validForColoring(vtkDataArray* a, bool& mapScalars);

signals:
  void currentFrameIDChanged(int);
  void modified();
  void colorOptionsEnabled(bool);

public slots:
  void showColorizeSurfaceMenu(int index);
  void reshowColorizeSurfaceMenu();
  void updateColorizeSurfaceMenu(QString const& text);

private:
  QTE_DECLARE_PRIVATE_RPTR(VolumeOptions)
  QTE_DECLARE_PRIVATE(VolumeOptions)

  QTE_DISABLE_COPY(VolumeOptions)
};

#endif
