#include <mobro/unicycle.h>
#include <Eigen/Dense>

void Unicycle::update(const rclcpp::Time &now)
{
  const auto c{cos(theta)};
  const auto s{sin(theta)};

  const auto [pd,vd,ad,use_traj] = currentGoal(now); {}

  // compute command
  if(gains.control == ControlType::LYAPUNOV)
  {
    const auto [xe, ye, te, vref, wref] = LyapunovError(pd, vd, ad); {}

    u[0] = vref*cos(te) + gains.Kx*xe;
    u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te;
  }
  else if(gains.control == ControlType::DYNAMIC_FB)
  {
    const auto v{linearVelocity(false)};    
    K(0,0) = c;
    K(0,1) = -v[1];
    K(1,0) = s;
    K(1,1) = v[0];

    // acceleration command
    const auto u_tilde{K.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(ad + gains.Kp*(pd-xy) + gains.Kv*(vd-v))};

    std::cout << u_tilde.transpose() << std::endl;

    // to velocity
    u[0] += u_tilde[0]*dt;
    u[1] = u_tilde[1];
  }
  else if(gains.control == ControlType::MPC)
  {
    const auto start{Clock::now()};

    Eigen::Vector3d cur({xy[0], xy[1], theta});
    mpc.configure(gains.horizon, gains.dt, cur, gains.vmax, gains.wmax);

    if(use_traj)
      u = mpc.compute(traj, now.seconds(), dt_ms);
    else
      u = mpc.compute(pd, dt_ms);

    std::cout << "MPC computation: " << microseconds_since(start)/1000 << " ms" << std::endl;

  }
  else // first order
  {
    Vector2d xyp = xy;
    xyp[0] += c*gains.d;
    xyp[1] += s*gains.d;

    K(0,0) = c;
    K(0,1) = -gains.d*s;
    K(1,0) = s;
    K(1,1) = gains.d*c;

    u = K.colPivHouseholderQr().solve(vd + gains.K*(pd-xyp));
  }

  u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);
  u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);

  // to cmd_vel
  cmd_vel.linear.x = u[0];
  cmd_vel.angular.z = u[1];

  publish(pd, now);
}
