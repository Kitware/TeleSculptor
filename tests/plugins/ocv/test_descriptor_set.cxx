/*ckwg +29
 * Copyright 2013-2014 by Kitware, Inc.
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
 * \brief test OCV descriptor_set class
 */

#include <test_common.h>

#include <maptk/plugins/ocv/descriptor_set.h>

#define TEST_ARGS ()

DECLARE_TEST_MAP();

int
main(int argc, char* argv[])
{
  CHECK_ARGS(1);

  testname_t const testname = argv[1];

  RUN_TEST(testname);
}

using namespace kwiver::vital;

IMPLEMENT_TEST(default_set)
{
  using namespace maptk;
  ocv::descriptor_set ds;
  if (ds.size() != 0)
  {
    TEST_ERROR("Default descriptor_set is not empty");
  }
  if (!ds.descriptors().empty())
  {
    TEST_ERROR("Default descriptor_set produces non-empty descriptors");
  }
}


IMPLEMENT_TEST(populated_set)
{
  using namespace maptk;
  const unsigned num_desc = 100;
  const unsigned dim = 128;
  cv::Mat data(num_desc, dim, CV_64F);
  cv::randu(data, 0, 1);
  ocv::descriptor_set ds(data);
  if (ds.size() != num_desc)
  {
    TEST_ERROR("descriptor_set is not the expected size");
  }
  if (ds.ocv_desc_matrix().data != data.data)
  {
    TEST_ERROR("descriptor_set does not contain original cv::Mat");
  }
  std::vector<descriptor_sptr> desc = ds.descriptors();
  if (desc.size() != ds.size())
  {
    TEST_ERROR("descriptor_set does not produce expected number of descriptors");
  }
  for( unsigned i=0; i<num_desc; ++i)
  {
    if (desc[i]->size() != dim)
    {
      TEST_ERROR("some descriptors do not have the correct dimension");
      break;
    }
    std::vector<double> vals = desc[i]->as_double();
    cv::Mat row = data.row(i);
    if (!std::equal(vals.begin(), vals.end(), row.begin<double>()))
    {
      TEST_ERROR("Floating point values of decriptors are not correct");
      break;
    }
  }
}


namespace {
void test_conversions(const cv::Mat& data)
{
  using namespace maptk;
  static std::string type_names[] = {"ubyte", "byte", "ushort", "short",
                                     "int", "float", "double", "unknown"};
#define CURRENT_TYPE "descriptor_set ("<< type_names[data.type()%8] \
                                       <<": "<<data.size() << ")"
  ocv::descriptor_set ds(data);
  if (static_cast<int>(ds.size()) != data.rows)
  {
    TEST_ERROR(CURRENT_TYPE " is not the expected size");
  }
  std::vector<descriptor_sptr> desc = ds.descriptors();
  if (desc.size() != ds.size())
  {
    TEST_ERROR(CURRENT_TYPE " does not produce expected number of descriptors");
  }
  cv::Mat double_data;
  data.convertTo(double_data, CV_64F);
  for( unsigned i=0; i<desc.size(); ++i)
  {
    if (static_cast<int>(desc[i]->size()) != data.cols)
    {
      TEST_ERROR(CURRENT_TYPE " some descriptors do not have the correct dimension");
      break;
    }
    std::vector<double> vals = desc[i]->as_double();
    std::vector<byte> byte_vals = desc[i]->as_bytes();
    if (desc[i]->num_bytes() != byte_vals.size())
    {
      TEST_ERROR(CURRENT_TYPE " some descriptors do not have the correct number of bytes");
      break;
    }
    cv::Mat row = double_data.row(i);
    if (!std::equal(vals.begin(), vals.end(), row.begin<double>()))
    {
      TEST_ERROR(CURRENT_TYPE " floating point values of decriptors are not correct");
      break;
    }
  }

  simple_descriptor_set simp_ds(desc);
  cv::Mat recon_mat = ocv::descriptors_to_ocv_matrix(simp_ds);
  if (recon_mat.data == data.data)
  {
    TEST_ERROR(CURRENT_TYPE " reconstructed matrix points to original, not new memory");
  }
  if (recon_mat.type() != data.type() ||
      recon_mat.size() != data.size() ||
      cv::countNonZero(recon_mat != data) > 0)
  {
    TEST_ERROR(CURRENT_TYPE " reconstructed matrix does not equal original");
  }
}

inline cv::Mat rand_double_mat(int r, int c)
{
  cv::Mat m(r, c, CV_64F);
  cv::randu(m, 0.0, 1.0);
  return m;
}

inline cv::Mat rand_float_mat(int r, int c)
{
  cv::Mat m(r, c, CV_32F);
  cv::randu(m, 0.0f, 1.0f);
  return m;
}

inline cv::Mat rand_byte_mat(int r, int c)
{
  cv::Mat m(r, c, CV_8U);
  cv::randu(m, 0, 255);
  return m;
}

}


IMPLEMENT_TEST(double_conversions)
{
  test_conversions(rand_double_mat(1, 50));
  test_conversions(rand_double_mat(64, 50));
  test_conversions(rand_double_mat(128, 1));
  test_conversions(rand_double_mat(125, 20));
  test_conversions(rand_double_mat(256, 10));
}


IMPLEMENT_TEST(float_conversions)
{
  test_conversions(rand_float_mat(1, 50));
  test_conversions(rand_float_mat(64, 50));
  test_conversions(rand_float_mat(128, 1));
  test_conversions(rand_float_mat(125, 20));
  test_conversions(rand_float_mat(256, 10));
}


IMPLEMENT_TEST(byte_conversions)
{
  test_conversions(rand_byte_mat(1, 50));
  test_conversions(rand_byte_mat(64, 50));
  test_conversions(rand_byte_mat(128, 1));
  test_conversions(rand_byte_mat(125, 20));
  test_conversions(rand_byte_mat(256, 10));
}
