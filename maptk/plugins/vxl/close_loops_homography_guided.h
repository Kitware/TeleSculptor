/*ckwg +29
 * Copyright 2014-2015 by Kitware, Inc.
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
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
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

/**
 * \file
 * \brief Header defining the vxl \link maptk::vxl::close_loops_homography_guided
 *        close_loops \endlink algorithm
 */

#ifndef MAPTK_PLUGINS_VXL_CLOSE_LOOPS_HOMOGRAPHY_GUIDED_H_
#define MAPTK_PLUGINS_VXL_CLOSE_LOOPS_HOMOGRAPHY_GUIDED_H_

#include <boost/scoped_ptr.hpp>

#include <vital/types/image_container.h>
#include <vital/types/track_set.h>

#include <vital/algo/close_loops.h>
#include <maptk/plugins/vxl/vxl_config.h>


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
class MAPTK_VXL_EXPORT close_loops_homography_guided
  : public kwiver::vital::algorithm_impl<vxl::close_loops_homography_guided, kwiver::vital::algo::close_loops>
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

  /// Get this algorithm's \link kwiver::vital::config_block configuration block \endlink
  /**
   * This base virtual function implementation returns an empty configuration
   * block whose name is set to \c this->type_name.
   *
   * \returns \c kwiver::vital::config_block containing the configuration for this algorithm
   *          and any nested components.
   */
  virtual kwiver::vital::config_block_sptr get_configuration() const;

  /// Set this algorithm's properties via a config block
  /**
   * \throws no_such_configuration_value_exception
   *    Thrown if an expected configuration value is not present.
   * \throws algorithm_configuration_exception
   *    Thrown when the algorithm is given an invalid \c kwiver::vital::config_block or is'
   *    otherwise unable to configure itself.
   *
   * \param config  The \c kwiver::vital::config_block instance containing the configuration
   *                parameters for this algorithm
   */
  virtual void set_configuration( kwiver::vital::config_block_sptr config );

  /// Check that the algorithm's currently configuration is valid
  /**
   * This checks solely within the provided \c kwiver::vital::config_block and not against
   * the current state of the instance. This isn't static for inheritence
   * reasons.
   *
   * \param config  The config block to check configuration of.
   *
   * \returns true if the configuration check passed and false if it didn't.
   */
  virtual bool check_configuration( kwiver::vital::config_block_sptr config ) const;

  /// Perform loop closure operation.
  /**
   * \param frame_number the frame number of the current frame
   * \param input the input track set to stitch
   * \param image image data for the current frame
   * \param mask Optional mask image where positive values indicate
   *                  regions to consider in the input image.
   * \returns an updated set a tracks after the stitching operation
   */
  virtual kwiver::vital::track_set_sptr
  stitch( kwiver::vital::frame_id_t frame_number,
          kwiver::vital::track_set_sptr input,
          kwiver::vital::image_container_sptr image,
          kwiver::vital::image_container_sptr mask = kwiver::vital::image_container_sptr() ) const;

private:

  /// Class for storing other internal variables
  class priv;
  boost::scoped_ptr<priv> d_;

};


} // end namespace vxl

} // end namespace maptk


#endif // MAPTK_PLUGINS_VXL_CLOSE_LOOPS_HOMOGRAPHY_GUIDED_H_
