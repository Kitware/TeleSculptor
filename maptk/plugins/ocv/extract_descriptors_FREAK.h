//
// Created by Paul Tunison on 3/27/16.
//

#ifndef MAPTK_HAS_OPENCV_VER_3
#ifndef MAPTK_EXTRACT_DESCRIPTORS_FREAK_H_
#define MAPTK_EXTRACT_DESCRIPTORS_FREAK_H_

#include <memory>
#include <string>

#include <maptk/plugins/ocv/extract_descriptors.h>
#include <maptk/plugins/ocv/maptk_ocv_export.h>

namespace kwiver {
namespace maptk {
namespace ocv {


class MAPTK_OCV_EXPORT extract_descriptors_FREAK
    : public kwiver::vital::algorithm_impl< extract_descriptors_FREAK,
                                            extract_descriptors,
                                            vital::algo::extract_descriptors >
{
public:
  /// Constructor
  extract_descriptors_FREAK();

  /// Copy Constructor
  /**
   * \param other The other FREAK descriptor extractor to copy
   */
  extract_descriptors_FREAK(extract_descriptors_FREAK const &other);

  /// Destructor
  virtual ~extract_descriptors_FREAK();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "ocv_FREAK"; }
  /// Returns a descriptive string for this implementation
  virtual std::string description() const {
    return "OpenCV feature-point descriptor extraction via the FREAK algorithm";
  }

  /// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
  virtual vital::config_block_sptr get_configuration() const;
  /// Set this algorithm's properties via a config block
  virtual void set_configuration(vital::config_block_sptr config);
  /// Check that the algorithm's configuration config_block is valid
  virtual bool check_configuration(vital::config_block_sptr config) const;

private:
  /// private implementation class
  class priv;
  std::unique_ptr<priv> const p_;
};


}
}
}

#endif //MAPTK_EXTRACT_DESCRIPTORS_FREAK_H_
#endif //MAPTK_HAS_OPENCV_VER_3
