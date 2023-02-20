#include <mobro/unicycle.h>
#include <Eigen/Dense>

void Unicycle::computeMPC(const Goal &goal, const rclcpp::Time &now)
{
  Eigen::Vector3d cur({xy[0], xy[1], theta});
  mpc.configure(gains.horizon, gains.dt, cur, gains.vmax, gains.wmax);

  if(goal.use_traj)
    u = mpc.compute(traj, now.seconds(), dt_ms);
  else
    u = mpc.compute(goal.pd, dt_ms);
}

