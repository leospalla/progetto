#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "vector.hpp"

TEST_CASE("testing operators")
{
  Vector v1{2., 2.};
  Vector v2{1., 3.};
  SUBCASE("+ operator")
  {
    Vector v3= v1+v2;
    CHECK(v3.xcomp() == doctest::Approx(3.));
    CHECK(v3.ycomp() == doctest::Approx(5.));
  }
}
// da aggiungere gli altri test

