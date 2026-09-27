#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>
#include <numeric>
#include <algorithm>
#include <cfloat>
#include <vector>
#include "shulib/hal/motor_group.hpp"
using shulib::hal::medianInPlace;
static double med(std::vector<double> v){ return medianInPlace(v.data(), v.size()); }
static void pr(const char* n, double x){ unsigned long long b; std::memcpy(&b,&x,8); std::printf("%-34s %.17g  0x%016llx\n", n, x, b); }
int main(){
  const double I = std::numeric_limits<double>::infinity();
  const double N = std::numeric_limits<double>::quiet_NaN();
  pr("{x,x,NaN,x,x} x=3", med({3,3,N,3,3}));
  pr("{x,x,+Inf,x,x}", med({3,3,I,3,3}));
  pr("{x,x,-Inf,x,x}", med({3,3,-I,3,3}));
  pr("{1,+Inf,+Inf}", med({1,I,I}));
  pr("{1,2,+Inf,+Inf}", med({1,2,I,I}));
  pr("{1,+Inf}", med({1,I}));
  pr("{5,Inf,1,2,3}", med({5,I,1,2,3}));
  pr("{+Inf,-Inf}", med({I,-I}));
  double dm = std::numeric_limits<double>::denorm_min();
  pr("legacy {dm,dm}", med({dm,dm}));
  pr("0.5a+0.5b {dm,dm}", 0.5*dm+0.5*dm);
  pr("midpoint {dm,dm}", std::midpoint(dm,dm));
  pr("legacy {DBL_MAX,DBL_MAX}", med({DBL_MAX,DBL_MAX}));
  pr("midpoint {DBL_MAX,DBL_MAX}", std::midpoint(DBL_MAX,DBL_MAX));
  // calibrator spread at :1255-1257 as written
  auto spread=[&](std::vector<double> v){ double lo=0,hi=0; for(size_t k=0;k<v.size();++k){ lo=(k==0||v[k]<lo)?v[k]:lo; hi=(k==0||v[k]>hi)?v[k]:hi;} return hi-lo; };
  pr("spread {10,+Inf,12}", spread({10,I,12}));
  pr("spread {+Inf,10,12}", spread({I,10,12}));
  pr("spread {+Inf,+Inf}", spread({I,I}));
  // temperature max order
  pr("std::max(NaN,40)", std::max(N,40.0));
  pr("std::max(40,NaN)", std::max(40.0,N));
  // B7 two-member side
  pr("{0,NaN} (100 is NaN)", 0.0); 
  return 0;
}
