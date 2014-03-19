/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 * \brief Header defining the vxl \link maptk::vxl::close_loops_homography_guided
 *        close_loops \endlink algorithm
 */

#ifndef MAPTK_VXL_CLOSE_LOOPS_HOMOGRAPHY_GUIDED_H_
#define MAPTK_VXL_CLOSE_LOOPS_HOMOGRAPHY_GUIDED_H_

#include <maptk/core/core_config.h>
#include <maptk/core/image_container.h>
#include <maptk/core/track_set.h>

#include <boost/scoped_ptr.hpp>

#include <maptk/core/algo/algorithm.h>
#include <maptk/core/algo/close_loops_bad_frames_only.h>


namespace maptk
{

namespace vxl
{

/// Attempts to stitch tracks over a long period of time.
/**
 * This class attempts to make longer-term loop closures by utilizing a
 * variety of techniques, one of which involves using homographies to
 * estimate potential match locations in the past, followed up by additional
 * filtering.
 */
class MAPTK_CORE_EXPORT close_loops_homography_guided
  : public algo::algorithm_impl<vxl::close_loops_homography_guided, algo::close_loops>
{
public:

  /// Default Constructor
  close_loops_homography_guided();

  /// Copy Constructor
  close_loops_homography_guided( const close_loops_homography_guided& );

  /// Destructor
  virtual ~close_loops_homography_guided();

  /// Return the name of this implementation
  virtual std::string impl_name() const { return "vxl_homography_guided"; }

  /// Get this algorithm's \link maptk::config_block configuration block \endlink
  /**
   * This base virtual function implementation returns an empty configuration
   * block whose name is set to \c this->type_name.
   *
   * \returns \c config_block containing the configuration for this algorithm
   *          and any nested components.
   */
  virtual config_block_sptr get_configuration() const;

  /// Set this algorithm's properties via a config block
  /**
   * \throws no_such_configuration_value_exception
   *    Thrown if an expected configuration value is not present.
   * \throws algorithm_configuration_exception
   *    Thrown when the algorithm is given an invalid \c config_block or is'
   *    otherwise unable to configure itself.
   *
   * \param config  The \c config_block instance containing the configuration
   *                parameters for this algorithm
   */
  virtual void set_configuration( config_block_sptr config );

  /// Check that the algorithm's currently configuration is valid
  /**
   * This checks solely within the provided \c config_block and not against
   * the current state of the instance. This isn't static for inheritence
   * reasons.
   *
   * \param config  The config block to check configuration of.
   *
   * \returns true if the configuration check passed and false if it didn't.
   */
  virtual bool check_configuration( config_block_sptr config ) const;

  /// Perform loop closure operation.
  /**
   * \param [in] frame_number the frame number of the current frame
   * \param [in] image image data for the current frame
   * \param [in] input the input track set to stitch
   * \returns an updated set a tracks after the stitching operation
   */
  virtual track_set_sptr
  stitch( frame_id_t frame_number,
          image_container_sptr image,
          track_set_sptr input ) const;

private:

  /// Class for storing other internal variables
  class priv;
  boost::scoped_ptr<priv> d_;

};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_VXL_CLOSE_LOOPS_HOMOGRAPHY_GUIDED_H_
