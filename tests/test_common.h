/*ckwg +5
 * Copyright 2011-2013 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

/**
 * \file
 *
 * \brief Macro definitions for creating and running test cases
 *
 * These integrate with the paired CTest infrastucture managed by MAPTK's
 * CMake.
 */

#ifndef MAPTK_TEST_TEST_COMMON_H
#define MAPTK_TEST_TEST_COMMON_H

#include <boost/function.hpp>

#include <exception>
#include <iostream>
#include <map>
#include <string>

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
    typedef boost::function<void TEST_ARGS> test_function_t;  \
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
 * @sa TEST/home/purg/dev/perseas/test_runs/test-d2d_conductor_wrapper/test_output_loc/1111/activities_PROPERTY
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
// Testing helper methods
//

/// Equality test with message generation on inequality
/**
 * @param name
 */
#define TEST_EQUAL(name, value, expected)                                 \
  do                                                                      \
  {                                                                       \
    if(value != expected)                                                 \
    {                                                                     \
      TEST_ERROR("TEST_EQUAL check '" << name << "' failed." << std::endl \
                 << "    Expected: " << expected << std::endl             \
                 << "    Got     : " << value);                           \
    }                                                                     \
  } while(false)

/// Test double/float approximate equality to a given epsilon
/**
 * @param name    An identifying name for the test.
 * @param value   The value subject for comparison.
 * @param target  The value to compare to.
 * @param epsilon The allowed varience.
 */
static void TEST_NEAR(char   const *name,
                      double const &value,
                      double const &target,
                      double const &epsilon)
{
  if(fabs(value - target) > epsilon)
  {
    TEST_ERROR("TEST_NEAR check '" << name << "' failed:" << std::endl
               << "    Expected: " << target << std::endl
               << "    Got     : " << value  << std::endl
               << "    (epsilon: " << epsilon << ")");
  }
}

#endif // MAPTK_TEST_TEST_COMMON_H
