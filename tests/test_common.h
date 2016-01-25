/*ckwg +29
 * Copyright 2011-2016 by Kitware, Inc.
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
 *
 * \brief Macro definitions for creating and running test cases
 *
 * These integrate with the paired CTest infrastucture managed by MAPTK's
 * CMake.
 */

#ifndef MAPTK_TEST_TEST_COMMON_H_
#define MAPTK_TEST_TEST_COMMON_H_

#include <vital/vital_foreach.h>

#include <exception>
#include <iostream>
#include <map>
#include <string>
#include <functional>
#include <cstdlib>
#include <cstdio>
#include <cmath>

typedef std::string testname_t;

/// Report an error to stderr.
/**
 * @param msg The message to report.
 */
#define TEST_ERROR(msg)                         \
  do                                            \
  {                                             \
    std::cerr << "Error: " << msg << std::endl; \
  } while (false)

/// Attempt a code block that should throw some exception
/**
 * @param ex      Exception class or type to expect.
 * @param code    Block of code to execute in which we expect the given
 *                exception.
 * @param action  Message describing the action that should have caused
 *                the expected exception.
 */
#define EXPECT_EXCEPTION(ex, code, action)  \
  do                                        \
  {                                         \
    bool got_exception = false;             \
                                            \
    try                                     \
    {                                       \
      code;                                 \
    }                                       \
    catch (ex const& e)                     \
    {                                       \
      got_exception = true;                 \
                                            \
      std::cerr << "Expected exception: "   \
                << e.what()                 \
                << std::endl;               \
    }                                       \
    catch (std::exception const& e)         \
    {                                       \
      TEST_ERROR("Unexpected exception: "   \
                 << e.what());              \
                                            \
      got_exception = true;                 \
    }                                       \
    catch (...)                             \
    {                                       \
      TEST_ERROR("Non-standard exception"); \
                                            \
      got_exception = true;                 \
    }                                       \
                                            \
    if (!got_exception)                     \
    {                                       \
      TEST_ERROR("Did not get "             \
                 "expected exception when " \
                 << action);                \
    }                                       \
  } while (false)


/// Set-up macro defining the test case function map for the current file
/**
 * This *MUST* be declared once at the top of every file containing test case
 * functions.
 *
 * \a TEST_ARGS must be defined before this call, declaring the function
 * argument signature for test cases declared in the file, i.e. if cases
 * needed to take in a path to a data directory or the like.
 */
#define DECLARE_TEST_MAP()                                    \
  namespace                                                   \
  {                                                           \
    typedef std::function<void TEST_ARGS> test_function_t;    \
    typedef std::map<testname_t, test_function_t> test_map_t; \
  }                                                           \
  test_map_t __all_tests;                                     \
  struct __add_test                                           \
  {                                                           \
    __add_test(testname_t const& name,                        \
               test_function_t const& func)                   \
    {                                                         \
      __all_tests[name] = func;                               \
    }                                                         \
  }                                                           \

/// Macro for displaying tests available
#define DISPLAY_AVAILABLE_TESTS()                           \
  do                                                        \
  {                                                         \
    std::cerr << "Available tests:" << std::endl;           \
    VITAL_FOREACH( test_map_t::value_type p, __all_tests )  \
    {                                                       \
      std::cerr << "\t" << p.first << std::endl;            \
    }                                                       \
  } while (false)

/// Add a CMake property to the next test declared
/**
 * This is a hook for the CMake parsing code to set CTest test properties via
 * the ``set_tests_properties(...)`` CMake method. Properties declared are set
 * on the next test declared. The special property \a ENVIRONMENT can only be
 * set once. Subsiquent TEST_PROPERTY calls setting \a ENVIRONMENT overwrite
 * previous set attempts.
 *
 * @param property  The CMake test property to set.
 * @param value     The value to set to the CMake test property.
 *
 * @sa IMPLEMENT_TEST(testname)
 */
#define TEST_PROPERTY(property, value, ...)

/// Define a test case
/**
 * @param testname  The name of the test case to define.
 * @sa TEST_PROPERTY
 */
#define IMPLEMENT_TEST(testname)                       \
  static void                                          \
  test_##testname TEST_ARGS;                           \
  static __add_test const                              \
    __add_test_##testname(#testname, test_##testname); \
  void                                                 \
  test_##testname TEST_ARGS

/// Check the number of positional arguments given to the top level executable
/**
 * @param numargs The number of arguments to expect after the name of the
 *                executable.
 */
#define CHECK_ARGS(numargs)     \
  do                            \
  {                             \
    if (argc != (numargs + 1))  \
    {                           \
      TEST_ERROR("Expected "    \
                 #numargs       \
                 " arguments"); \
                                \
      std::cerr << std::endl;   \
      DISPLAY_AVAILABLE_TESTS();\
      std::cerr << std::endl;   \
                                \
      return EXIT_FAILURE;      \
    }                           \
  } while (false)

/// Run the a test case by a given name
/**
 * Find an run the test function associated with the given testname.
 * Parameters after the test name are the arguments to pass to the function.
 *
 * @param testname  The name of the test to run. This name should match one
 *                  given to an IMPLEMENT_TEST() macro.
 *
 * @sa DECLARE_TEST_MAP(), IMPLEMENT_TEST(), CHECK_ARGS()
 */
#define RUN_TEST(testname, ...)                 \
  do                                            \
  {                                             \
    test_map_t::const_iterator const i =        \
      __all_tests.find(testname);               \
                                                \
    if (i == __all_tests.end())                 \
    {                                           \
      TEST_ERROR("Unknown test: " << testname); \
                                                \
      std::cerr << std::endl;                   \
      DISPLAY_AVAILABLE_TESTS();                \
      std::cerr << std::endl;                   \
                                                \
      return EXIT_FAILURE;                      \
    }                                           \
                                                \
    test_function_t const& func = i->second;    \
                                                \
    try                                         \
    {                                           \
      func(__VA_ARGS__);                        \
    }                                           \
    catch (std::exception const& e)             \
    {                                           \
      TEST_ERROR("Unexpected exception: "       \
                 << e.what());                  \
                                                \
      return EXIT_FAILURE;                      \
    }                                           \
                                                \
    return EXIT_SUCCESS;                        \
  } while (false)


//
// Testing helper macros/methods
//

namespace kwiver {
namespace maptk {
namespace testing {

/// Test double approximate equality to given epsilon
/**
 * @param value   The value subject for comparison.
 * @param target  The value to compare to.
 * @param epsilon The allowed varience.
 */
inline bool is_almost(double const &value,
                      double const &target,
                      double const &epsilon)
{
  return fabs(value - target) <= epsilon;
}

} //end namespace testing
} //end namespace maptk
} //end namespace kwiver

/// General equality test with message generation on inequality
/**
 * Test equality between values with a '!=' expression. This wrapps a standard
 * error response message.
 *
 * @param name      A descriptive name for this specific test.
 * @param value     The experimental value of the equality check.
 * @param expected  The expected value of the equality check.
 */
#define TEST_EQUAL(name, value, expected)                       \
  do                                                            \
  {                                                             \
    if((value) != (expected))                                   \
    {                                                           \
      TEST_ERROR("TEST_EQUAL check '" << name << "' failed:\n"  \
                 << "    Expected: ``" << (expected) << "``\n"  \
                 << "    Got     : ``" << (value) << "``");     \
    }                                                           \
  } while(false)

/// Test double/float approximate equality to a given epsilon
/**
 * @param name    An identifying name for the test.
 * @param value   The value subject for comparison.
 * @param target  The value to compare to.
 * @param epsilon The allowed varience.
 */
#define TEST_NEAR(name, value, target, epsilon)                  \
  do                                                             \
  {                                                              \
    namespace kmt = kwiver::maptk::testing;                      \
    if(! kmt::is_almost(value, target, epsilon))                 \
    {                                                            \
      TEST_ERROR("TEST_NEAR check '" << name                     \
                 << "' failed: (epsilon: " << (epsilon) << ")\n" \
                 << "    Expected: " << (target) << "\n"         \
                 << "    Got     : " << (value));                \
    }                                                            \
  }while(false)

#endif // MAPTK_TEST_TEST_COMMON_H_
