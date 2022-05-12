#include <mobro/robot.h>

#ifdef USE_GA
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

#ifdef USE_CT
#include <mobro/mpc_ct.h>

class UnicycleModel : public ct::core::ControlledSystem<3, 2>
{
public:
  inline ct::core::ControlledSystem<3,2>* clone() const override
  {
    return new UnicycleModel(*this);
  }

  inline void computeControlledDynamics(const ct::core::StateVector<3>& state,
          [[maybe_unused]] const time_t& t,
          const ct::core::ControlVector<2>& control,
          ct::core::StateVector<3>& derivative) override
  {
    const auto c{cos(state(2))};
    const auto s{sin(state(2))};

    derivative(0) = control(0)*c;
    derivative(1) = control(0)*s;
    derivative(2) = control(1);
  }
};
#endif

#ifdef USE_CROCODDYL
#include <mobro/mpc_croc.h>
#include <crocoddyl/core/actions/unicycle.hpp>

class ActionModelUnicycle : public ActionModel<3>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = crocoddyl::ActionModelAbstractTpl<double>;
  typedef crocoddyl::ActionDataAbstractTpl<double> ActionDataAbstract;

  void calc(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x,
                    const Eigen::Ref<const VectorXd>& u)
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
                        const Eigen::Ref<const VectorXd>& u)
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

#endif



class Unicycle : public Robot
{
public:
  explicit Unicycle(rclcpp::Node::SharedPtr node, const Traj &traj) : Robot(node, traj, 2)
  #ifdef USE_CT
    , mpc(model)
  #endif
  #ifdef USE_CROCODDYL
    , mpc(new ActionModelUnicycle())
  #endif
  {
  }
  void update(const rclcpp::Time &now) override;

protected:
   MPCProblem<3> prob;
#ifdef USE_CT
  std::shared_ptr<UnicycleModel> model{std::make_shared<UnicycleModel>()};
  MobileMPC<3> mpc;
 #endif

#ifdef USE_GA
  GeneticMPC<3, UnicycleCandidate> mpc;
#endif

#ifdef USE_CROCODDYL
  MPCrocoddyl<3> mpc;
#endif
};
