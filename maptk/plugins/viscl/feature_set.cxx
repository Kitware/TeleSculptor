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


#include "feature_set.h"

#include <maptk/plugins/viscl/utils.h>

#include <viscl/core/manager.h>

#include <memory>

namespace kwiver {
namespace maptk {

namespace vcl
{


/// Return a vector of feature shared pointers
std::vector<vital::feature_sptr>
feature_set
::features() const
{
  std::vector<vital::feature_sptr> features;

  unsigned int numkpts = size();
  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();

  std::vector<cl_int2> kpts;
  kpts.resize(numkpts);
  queue->enqueueReadBuffer(*data_.features_().get(), CL_TRUE, 0, sizeof(cl_int2)*numkpts, &kpts[0]);

  //These features only have locations set for them
  for (unsigned int i = 0; i < numkpts; i++)
  {
    vital::feature_f *f = new vital::feature_f();
    f->set_loc(vital::vector_2f(kpts[i].s[0], kpts[i].s[1]));
    features.push_back(vital::feature_sptr(f));
  }

  return features;
}


// Convert any feature set to a VisCL data (upload if needed)
feature_set::type
features_to_viscl(const vital::feature_set& features)
{
  //if already on GPU in VisCL format, then access it
  if( const vcl::feature_set* f =
          dynamic_cast<const vcl::feature_set*>(&features) )
  {
    return f->viscl_features();
  }

  unsigned int width, height;
  min_image_dimensions(features, width, height);

  //kpt map is half width and height
  unsigned int ni, nj;
  ni = (width >> 1) + 1;
  nj = (height >> 1) + 1;
  size_t size = ni * nj;
  int *kp_map = new int [size];
  for (unsigned int i = 0; i < size; i++)
    kp_map[i] = -1;

  feature_set::type fs;
  int numfeat[1];
  numfeat[0] = static_cast<int>(features.size());
  fs.features_ = viscl::manager::inst()->create_buffer<cl_int2>(CL_MEM_READ_WRITE, numfeat[0]);
  fs.numfeat_ = viscl::manager::inst()->create_buffer<int>(CL_MEM_READ_WRITE, 1);

  //write number of features
  viscl::cl_queue_t queue = viscl::manager::inst()->create_queue();
  queue->enqueueWriteBuffer(*fs.numfeat_().get(), CL_TRUE, 0, fs.numfeat_.mem_size(), numfeat);

  //write features
  std::vector<vital::feature_sptr> feat = features.features();
  cl_int2 *buf = new cl_int2[features.size()];
  typedef std::vector<vital::feature_sptr>::const_iterator feat_itr;
  unsigned int j = 0;
  for(feat_itr it = feat.begin(); it != feat.end(); ++it, j++)
  {
    const vital::feature_sptr f = *it;

    cl_int2 kp;
    kp.s[0] = static_cast<int>(f->loc()[0]);
    kp.s[1] = static_cast<int>(f->loc()[1]);
    buf[j] = kp;
    size_t index = (kp.s[0] >> 1) * nj + (kp.s[1] >> 1);
    assert(index < size);

    kp_map[index] = j;
  }

  queue->enqueueWriteBuffer(*fs.features_().get(), CL_TRUE, 0, fs.features_.mem_size(), buf);
  queue->finish();

  cl::ImageFormat kptimg_fmt(CL_R, CL_SIGNED_INT32);
  fs.kptmap_ = viscl::image(std::make_shared<cl::Image2D>(
                            viscl::manager::inst()->get_context(),
                            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                            kptimg_fmt,
                            ni,
                            nj,
                            0,
                            (void *)kp_map));

  delete [] buf;
  delete [] kp_map;

  return fs;
}

size_t
feature_set
::size() const
{
  int buf[1];
  viscl::cl_queue_t q = viscl::manager::inst()->create_queue();
  q->enqueueReadBuffer(*data_.numfeat_().get(), CL_TRUE, 0, data_.numfeat_.mem_size(), buf);
  return buf[0];
}


} // end namespace vcl

} // end namespace maptk
} // end namespace kwiver
