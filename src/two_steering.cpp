#include <mobro/two_steering.h>
#include <Eigen/Dense>

void TwoSteering::computeStaticFB(const Goal &goal)
{
  const auto [c,s,c1,s1,c2,s2,ct1,st1,ct2,st2] = CS(); {}

  Eigen::Vector3d zd, zdd;
  // reference
  zd << goal.pd, atan2(goal.vd[1], goal.vd[0]);
  zdd << goal.vd, 0;

  //  linear part
  Eigen::Vector3d zc;
  zc.head<2>() = xy;

  // angular
  const auto d{gains.d};
  zc[0] += L*c + ct1*d;
  zc[1] += L*s + st1*d;

  const auto xp2{xy[0] + ct2*d};
  const auto yp2{xy[1] + st2*d};
  zc[2] = atan2(zc[1]-yp2, zc[0]-xp2);

  K(0,0) = cos(beta1 + theta) - d*sin(beta1 - beta2)*sin(beta1 + theta)/(L*cos(beta2));
  K(0,1) = -d*sin(beta1 + theta);
  K(0,2) = 0;
  K(1,0) = (L*sin(beta1 + theta) + d*sin(beta1)*cos(beta1 + theta) - d*cos(beta1)*cos(beta1 + theta)*tan(beta2))/L;
  K(1,1) = d*cos(beta1 + theta);
  K(1,2) = 0;
  K(2,0) = sin(beta1 - beta2)/(L*cos(beta2));
  const auto denom{1./(pow(L, 2) + 2*L*d*cos(beta1) - 2*L*d*cos(beta2) - 2*pow(d, 2)*cos(beta1 - beta2) + 2*pow(d, 2))};
  K(2,1) = d*(L*cos(beta1) - d*cos(beta1 - beta2) + d) * denom;
  K(2,2) = d*(-L*cos(beta2) - d*cos(beta1 - beta2) + d) * denom;

  u = solve(K, zdd + gains.K*(zd-zc));
}
