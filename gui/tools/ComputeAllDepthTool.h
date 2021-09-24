// This file is part of TeleSculptor, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/TeleSculptor/blob/master/LICENSE for details.

#ifndef TELESCULPTOR_COMPUTEALLDEPTHTOOL_H_
#define TELESCULPTOR_COMPUTEALLDEPTHTOOL_H_

#include "AbstractTool.h"

class ComputeAllDepthToolPrivate;

class ComputeAllDepthTool : public AbstractTool
{
  Q_OBJECT

public:
  explicit ComputeAllDepthTool(QObject* parent = nullptr);
  virtual ~ComputeAllDepthTool();

  virtual Outputs outputs() const override;

  /// Get if the tool can be canceled.
  virtual bool isCancelable() const override { return true; }

  virtual bool execute(QWidget* window = nullptr) override;

  bool callback_handler(kwiver::vital::image_container_sptr depth,
                        std::string const& status,
                        unsigned int percent_complete,
                        kwiver::vital::image_container_sptr uncertainty);

  /// handler for callback on image gathering status
  bool gather_status_handler(unsigned int curr_frame,
                             unsigned int num_frames,
                             unsigned int curr_depth_map,
                             unsigned int num_depth_maps,
                             kwiver::vital::frame_id_t active_frame_id);

protected:
  virtual void run() QTE_OVERRIDE;

private:
  QTE_DECLARE_PRIVATE_RPTR(ComputeAllDepthTool)
  QTE_DECLARE_PRIVATE(ComputeAllDepthTool)
  QTE_DISABLE_COPY(ComputeAllDepthTool)
};

#endif
