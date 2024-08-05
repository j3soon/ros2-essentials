/**
 * @file /include/kobuki_core/macros.hpp
 *
 * @brief Macros for kobuki_core.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_CORE_MACROS_HPP_
#define KOBUKI_CORE_MACROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Declspecs
*****************************************************************************/

/*
 * Import/exports symbols for the library
 */
#ifdef ECL_HAS_SHARED_LIBS // ecl is being built around shared libraries
  #ifdef kobuki_EXPORTS // we are building a shared lib/dll
    #define kobuki_PUBLIC ECL_HELPER_EXPORT
    #define EXP_TEMPLATE
  #else // we are using shared lib/dll
    #define kobuki_PUBLIC ECL_HELPER_IMPORT
    #define EXP_TEMPLATE extern
  #endif
  #define kobuki_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define kobuki_PUBLIC
  #define kobuki_LOCAL
  #define EXP_TEMPLATE
#endif

#endif /* KOBUKI_CORE_MACROS_HPP_ */
