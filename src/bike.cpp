#include <mobro/bike.h>

void Bike::update(const rclcpp::Time &now)
{
  const auto c{cos(theta)};
  const auto s{sin(theta)};
  const auto cb{cos(beta)};
  const auto sb{sin(beta)};
  const auto ctb{cos(theta+beta)};
  const auto stb{sin(theta+beta)};

  const auto [pd,vd,ad,use_traj] = currentGoal(now); {}

  // compute command
  if(gains.control == ControlType::LYAPUNOV)
  {
    const auto [xe, ye, te, vref, wref] = LyapunovError(pd, vd, ad, L, beta); {}
    u[0] = vref*cos(te) + gains.Kx*xe;
    u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te - u[0]/L*sb;
  }
  else if(gains.control == ControlType::DYNAMIC_FB)
  {
    Vector2d xyp = xy;
    xyp[0] += L*c;
    xyp[1] += L*s;

    Vector2d vp = linearVelocity(false);
    vp[0] -= u[0]*stb;
    vp[1] += u[0]*c*sb;

    K(0,0) = ctb;
    K(0,1) = -u[0]*ctb;
    K(1,0) = stb;
    K(1,1) = u[0]*ctb;

    const auto u_tilde{K.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(ad + gains.Kp*(pd-xyp) + gains.Kv*(vd-vp))};

    // to velocity
    u[0] += u_tilde[0]*dt;
    u[1] = u_tilde[1];
  }
  else
  {
    Vector2d xyp = xy;
    xyp[0] += L*c + ctb*gains.d;
    xyp[1] += L*s + stb*gains.d;

    K(0,0) = ctb-gains.d/L*stb*sb;
    K(0,1) = -gains.d*stb;
    K(1,0) = stb+gains.d/L*ctb*sb;
    K(1,1) = gains.d*ctb;

    u = K.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vd + gains.K*(pd-xyp));
  }

  u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);
  u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);

  // to cmd_vel
  cmd_vel.linear.x = u[0]*cb;
  cmd_vel.angular.z = u[0]*sb/L;

  // to joints
  cmd.data[0] = u[0];
  cmd.data[1] = u[1];
  cmd_pub->publish(cmd);


  publish(pd, now);
}
