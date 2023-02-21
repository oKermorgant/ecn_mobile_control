#include <mobro/robot.h>
#include <mobro/mpc/mpc_ga.h>

struct UnicycleCandidate : public GACandidate<3>
{
  void update(Eigen::Matrix<double, 3, 1> &state, const Vector2d &ui, double dt) override
  {
    const auto c{cos(state(2))};
    const auto s{sin(state(2))};

    state(0) += ui[0]*c*dt;
    state(1) += ui[0]*s*dt;
    state(2) += ui[1]*dt;
  }
};


class Unicycle : public Robot
{
public:
  explicit Unicycle(rclcpp::Node::SharedPtr node, const Traj &traj) : Robot(node, traj, 2)
  {}

  void computeStaticFB(const Goal &goal)
  {
    const auto [c,s] = CS(); {}
    Vector2d xyp = xy;
    xyp[0] += c*gains.d;
    xyp[1] += s*gains.d;

    K(0,0) = c;
    K(0,1) = -gains.d*s;
    K(1,0) = s;
    K(1,1) = gains.d*c;

    u = solve(K, goal.vd + gains.K*(goal.pd-xyp));
  }

  void computeDynamicFB(const Goal &goal)
  {
    const auto v{linearVelocity(false)};
    const auto [c,s] = CS(); {}
    K(0,0) = c;
    K(0,1) = -v[1];
    K(1,0) = s;
    K(1,1) = v[0];

    // acceleration command
    const auto u_tilde{solve(K, goal.ad + gains.Kp*(goal.pd-xy) + gains.Kv*(goal.vd-v))};

    // to velocity
    u[0] += u_tilde[0]*dt;
    u[1] = u_tilde[1];
  }

  void computeLyapunovFB(const Goal &goal)
  {
    const auto [xe, ye, te, vref, wref] = LyapunovError(goal.pd, goal.vd, goal.ad); {}
    u[0] = vref*cos(te) + gains.Kx*xe;
    u[1] = wref + gains.Ky*ye*vref*sinc(te) + gains.Kt*te;
  }

  void computeMPC(const Goal &goal, const rclcpp::Time &now);

  void buildCmd(const rclcpp::Time &now)
  {
    u[0] = std::clamp(u[0], -gains.vmax, gains.vmax);
    u[1] = std::clamp(u[1], -gains.wmax, gains.wmax);
    // to cmd_vel
    cmd_vel.linear.x = u[0];
    cmd_vel.angular.z = u[1];
  }

protected:

  std::pair<double,double> CS() const {return {cos(theta), sin(theta)};}

   MPCProblem<3> prob;
  GeneticMPC<3, UnicycleCandidate> mpc;
};
