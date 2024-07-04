/**
 * @file /ecl_config/src/test/char_sign.cpp
 *
 * @brief Tests for ecl macros
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/config/macros.hpp"

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Macros, unused) {
  // without ECL_UNUSED, with the current project settings, this should cause
  // 'error: typedef 'foo' locally defined but not used on at least gcc
  typedef int ECL_UNUSED foo;
  SUCCEED();
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}


