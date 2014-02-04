/*ckwg +5
 * Copyright 2014 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */


#include <maptk/viscl/feature_set.h>
#include <viscl/core/manager.h>

namespace maptk
{

namespace vcl
{


/// Return a vector of feature shared pointers
std::vector<feature_sptr>
feature_set
::features() const
{
  std::vector<feature_sptr> features;

  viscl::cl_queue_t queue = queue = viscl::manager::inst()->create_queue();
  int buf[1];
  queue->enqueueReadBuffer(*data_.numfeat_().get(), CL_TRUE, 0, data_.numfeat_.mem_size(), buf);
  int numkpts = buf[0];

  std::vector<cl_int2> kpts;
  kpts.resize(numkpts);
  queue->enqueueReadBuffer(*data_.features_().get(), CL_TRUE, 0, sizeof(cl_int2)*numkpts, &kpts[0]);

  //These features only have locations set for them
  for (unsigned int i = 0; i < numkpts; i++)
  {
    feature_f *f = new feature_f();
    f->set_loc(vector_2f(kpts[i].s[0], kpts[i].s[1]));
    features.push_back(feature_sptr(f));
  }

  return features;
}


// Convert any feature set to a VisCL data (upload if needed)
/// width and height are the dimensions of the image that the features were
/// computed from - they are nessessary to create a search map for viscl tracker
/// viscl only cares about integer feature location, therefore you will lose info converting from
/// maptk feature set to viscl and back
feature_set::type
features_to_viscl(const maptk::feature_set& features, size_t ni, size_t nj)
{
  //if already on GPU in VisCL format, then access it
  if( const vcl::feature_set* f =
          dynamic_cast<const vcl::feature_set*>(&features) )
  {
    return f->viscl_features();
  }

  //kpt map is half width and height
  ni = ni >> 1;
  nj = nj >> 1;
  size_t size = ni * nj;
  int *kp_map = new int [size];
  for (unsigned int i = 0; i < size; i++)
    kp_map[i] = -1;

  feature_set::type fs;
  int numfeat[1];
  numfeat[0] = static_cast<int>(features.size());
  fs.features_ = viscl::manager::inst()->create_buffer<cl_int2>(CL_MEM_READ_WRITE, numfeat[0]);
  fs.numfeat_ = viscl::manager::inst()->create_buffer<int>(CL_MEM_READ_WRITE, 1);
  cl::ImageFormat kptimg_fmt(CL_R, CL_SIGNED_INT32);
  fs.kptmap_ = viscl::manager::inst()->create_image(kptimg_fmt, CL_MEM_READ_WRITE, ni, nj);

  //write number of features
  viscl::cl_queue_t queue = queue = viscl::manager::inst()->create_queue();
  queue->enqueueWriteBuffer(*fs.numfeat_().get(), CL_TRUE, 0, fs.numfeat_.mem_size(), numfeat);

  //write features
  std::vector<feature_sptr> feat = features.features();
  cl_int2 *buf = new cl_int2[features.size()];
  typedef std::vector<feature_sptr>::const_iterator feat_itr;
  unsigned int i = 0;
  for(feat_itr it = feat.begin(); it != feat.end(); ++it, i++)
  {
    const feature_sptr f = *it;

    cl_int2 kp;
    kp.s[0] = static_cast<int>(f->loc()[0]);
    kp.s[1] = static_cast<int>(f->loc()[1]);
    buf[i] = kp;

    kp_map[(kp.s[0] >> 1) * nj + (kp.s[1] >> 1)] = i;
  }

  queue->enqueueWriteBuffer(*fs.features_().get(), CL_TRUE, 0, fs.features_.mem_size(), buf);

  cl::size_t<3> origin;
  origin.push_back(0);
  origin.push_back(0);
  origin.push_back(0);

  cl::size_t<3> region;
  region.push_back(ni);
  region.push_back(nj);
  region.push_back(1);

  queue->enqueueWriteImage(*fs.kptmap_().get(), CL_TRUE, origin, region, 0, 0, (void *)kp_map);
  queue->finish();

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


} // end namespace viscl

} // end namespace maptk
