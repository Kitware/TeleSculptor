

/// Register algorithm implementations in this function
/**
 * Implementations of this method within a plugin should include a
 * ``.register_self()`` call for each algorithm implementation to be made
 * available.
 *
 * \returns The number of implementations that FAILED to register, i.e. a
 *          return of 0 means registration success.
 *
 * Remember to also use 'extern "C" ...' in plugin implementation if compiling
 * in C++ to prevent the dreaded symbol name-mangling.
 */
// Or:  - Return a vector of maptk::algo::algorithm_impl::register_self function pointers?
extern "C" int register_algo_impls(void);
