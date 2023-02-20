#include <mobro/robot.h>

#ifdef USE_CROCODDYL
#include <mobro/mpc/mpc_croc.h>
//#include <crocoddyl/core/actions/unicycle.hpp>

class ActionModelUnicycle : public ActionModel<3>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = crocoddyl::ActionModelAbstractTpl<double>;
  typedef crocoddyl::ActionDataAbstractTpl<double> ActionDataAbstract;

  void calc(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x,
                    const Eigen::Ref<const VectorXd>& u) override
  {
    auto d = static_cast<ActionData*>(data.get());
    const auto c{cos(x[2])};
    const auto s{sin(x[2])};

    d->xnext << x[0] + c * u[0] * dt, x[1] + s * u[0] * dt, x[2] + u[1] * dt;
    d->r.template head<2>() = x_cost*(x.head<2>()-xg);
    d->r[2] = 0.;
    d->r.template tail<2>() = u_cost * u;
    d->cost = .5 * d->r.dot(d->r);
  }

  void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x,
                        const Eigen::Ref<const VectorXd>& u) override
  {
    ActionModel<3>::calcDiff(data, x);

    auto d = static_cast<ActionData*>(data.get());
    const auto c{cos(x[2])};
    const auto s{sin(x[2])};
    const auto w_u{u_cost*u_cost};

    d->Lu = u * w_u;
    d->Luu.diagonal().setConstant(w_u);
    d->Fx(0, 2) = -s * u[0] * dt;
    d->Fx(1, 2) = c * u[0] * dt;
    d->Fu(0, 0) = c * dt;
    d->Fu(1, 0) = s * dt;
    d->Fu(2, 1) = dt;
  }
};

#else
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
#endif


class Unicycle : public Robot
{
public:
  explicit Unicycle(rclcpp::Node::SharedPtr node, const Traj &traj) : Robot(node, traj, 2)
  #ifdef USE_CROCODDYL
    , mpc(new ActionModelUnicycle())
  #endif
  {
  }

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
#ifdef USE_CROCODDYL
  MPCrocoddyl<ActionModelUnicycle> mpc;
#else
  GeneticMPC<3, UnicycleCandidate> mpc;
#endif
};
