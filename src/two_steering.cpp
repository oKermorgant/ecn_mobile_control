#include <mobro/two_steering.h>

void TwoSteering::update(const rclcpp::Time &now)
{
  const auto c{cos(theta)};
  const auto s{sin(theta)};
  const auto c1{cos(beta1)};
  const auto s1{sin(beta1)};
  const auto c2{cos(beta2)};
  const auto s2{sin(beta2)};
  const auto ct1{cos(theta + beta1)};
  const auto st1{sin(theta + beta1)};
  const auto ct2{cos(theta + beta2)};
  const auto st2{sin(theta + beta2)};

  const auto [pd,vd,ad,use_traj] = currentGoal(now); {}

  auto &v1{u[0]};

  // compute command
  if(gains.control == ControlType::LYAPUNOV)
  {
    u.setZero();
  /*  const auto [xe, ye, te, vref, wref] = LyapunovError(pd, vd, ad, L, d1); {}
    u[0] = vref*cos(te) + gains.Kx*xe;
    u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te - u[0]/L*sb;*/
  }
  else if(gains.control == ControlType::DYNAMIC_FB)
  {
    /*Vector2d xyp = xy;
    xyp[0] += L*c;
    xyp[1] += L*s;

    Vector2d vp = v;
    vp[0] -= u[0]*stb;
    vp[1] += u[0]*c*sb;

    K(0,0) = ctb;
    K(0,1) = -u[0]*ctb;
    K(1,0) = stb;
    K(1,1) = u[0]*ctb;

    const auto u_tilde{K.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(ad + gains.Kp*(pd-xyp) + gains.Kv*(vd-vp))};

    // to velocity
    u[0] += u_tilde[0]*dt;
    u[1] = u_tilde[1];*/
    u.setZero();
  }
  else
  {
    Eigen::Vector3d zd, zdd;
    // reference
    zd << pd, atan2(vd[1], vd[0]);
    zdd << vd, 0;

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

    u = K.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(zdd + gains.K*(zd-zc));
  }

  v1 = std::clamp(v1, -gains.vmax, gains.vmax);
  u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);
  u[2] = std::clamp(u[2], -gains.wmax, gains.wmax);

  const auto v2{v1*c1/c2};

  // to cmd vel
  cmd_vel.linear.x = v1*c1;
  cmd_vel.linear.y = v2*s2;
  cmd_vel.angular.z = (v1*s1-v2*s2)/L;

  // to joints
  cmd.data[0] = u[0];
  cmd.data[1] = u[1];
  cmd.data[2] = u[2];
  cmd_pub->publish(cmd);

  publish(pd, now);
}
