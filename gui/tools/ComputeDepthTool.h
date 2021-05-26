// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_COMPUTEDEPTHTOOL_H_
#define TELESCULPTOR_COMPUTEDEPTHTOOL_H_

#include "AbstractTool.h"

class ComputeDepthToolPrivate;

class ComputeDepthTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit ComputeDepthTool(QObject* parent = nullptr);
  ~ComputeDepthTool() override;

  Outputs outputs() const override;

  /// Get if the tool can be canceled.
  bool isCancelable() const override { return true; }

  bool execute(QWidget* window = nullptr) override;

  bool callback_handler(kwiver::vital::image_container_sptr depth,
                        std::string const& status,
                        unsigned int percent_complete,
                        kwiver::vital::image_container_sptr uncertainty);

  /// handler for callback on image gathering status
  bool gather_status_handler(unsigned int curr_frame,
                             unsigned int num_frames);

protected:
  void run() override;

private:
  QTE_DECLARE_PRIVATE_RPTR(ComputeDepthTool)
  QTE_DECLARE_PRIVATE(ComputeDepthTool)
  QTE_DISABLE_COPY(ComputeDepthTool)
};

///Convert a kwiver depth map to a colored vtk image with optional mask
vtkSmartPointer<vtkImageData>
depth_to_vtk(const kwiver::vital::image_of<double>& depth_img,
             const kwiver::vital::image_of<unsigned char>& color_img,
             int i0, int ni, int j0, int nj,
             const kwiver::vital::image_of<double>& uncertainty_img = {},
             const kwiver::vital::image_of<unsigned char>& mask_img = {});

#endif
