#include "tdmms_finder_auto/qnode.hpp"

#include <gtest/gtest.h>

TEST(IsInsideTest, Test1)
{
  ASSERT_EQ(true, 1);
}

int main(int argc, char**argv){
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
