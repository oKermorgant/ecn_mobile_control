#ifndef MPC_CROC_H
#define MPC_CROC_H

#include <mobro/mpc.h>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/states/euclidean.hpp>

//template <size_t state_dim>

struct ActionData : public crocoddyl::ActionDataAbstractTpl<double>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using MathBase = crocoddyl::MathBaseTpl<double>;
  using Base = crocoddyl::ActionDataAbstractTpl<double>;
  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xnext;

  template <class Model>
  explicit ActionData(Model* const model) : Base(model) {
    Fx.diagonal().array() = Scalar(1.);
  }
};

template <size_t state_dim>
class ActionModel : public crocoddyl::ActionModelAbstractTpl<double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = crocoddyl::ActionModelAbstractTpl<double>;
  typedef crocoddyl::ActionDataAbstractTpl<double> ActionDataAbstract;

  inline ActionModel() : Base(boost::make_shared<crocoddyl::StateVectorTpl<double>>(state_dim), 2, 2+state_dim)
  {}

  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x,
                    const Eigen::Ref<const VectorXd>& u) = 0;
  inline void calc(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x)
  {
    auto d{static_cast<ActionData*>(data.get())};
    d->r.template head<2>() = x_cost*(x.head<2>()-xg);
    d->r.template tail<3>().setZero();
    d->cost = .5 * d->r.template head<2>().dot(d->r.template head<2>());
  }
  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x,
                        const Eigen::Ref<const VectorXd>& u) = 0;
  inline void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXd>& x)
  {
    auto d = static_cast<ActionData*>(data.get());

    const auto w_x{x_cost*x_cost};
    d->Lx.head<2>() = (x.head<2>()-xg) * w_x;
    d->Lxx.diagonal().head<2>().setConstant(w_x);
  }
  inline boost::shared_ptr<ActionDataAbstract> createData()
  {
    return boost::allocate_shared<ActionData>(Eigen::aligned_allocator<ActionData>(), this);
  }
  inline bool checkData(const boost::shared_ptr<ActionDataAbstract>& data)
  {
    auto d = boost::dynamic_pointer_cast<ActionData>(data);
    return d != nullptr;
  }

  inline void getGoal(double x, double y)
  {
    xg(0) = x;
    xg(1) = y;
  }

  inline void configure(double x, double u, double dt)
  {
    x_cost = x;
    u_cost = u;
    this->dt = dt;
  }

protected:
  using Base::nu_;     //!< Control dimension
  using Base::state_;  //!< Model of the state

  double x_cost{10.};
  double u_cost{1.};
  double dt{0.1};
  Vector2d xg;
};


template <size_t state_dim>
class MPCrocoddyl : public MPC<state_dim>
{
  static constexpr size_t control_dim{2};
  using Model = ActionModel<state_dim>;
  using MPC<state_dim>::prob;
  boost::shared_ptr<Model> model;
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> runningModels;
  std::vector<Eigen::VectorXd> xs;
  std::vector<Eigen::VectorXd> us;

public:
  explicit MPCrocoddyl(Model* model) : model{model}
  {
  }

  Vector2d compute_impl(int max_ms, bool horizon_change) override
  {
    if(horizon_change)
    {
      const auto N{prob.size()};

      // resize our vals
      xs.resize(N);
      for(auto &x: xs)
        x = prob.state;
      us.resize(N, Vector2d::Zero());

      runningModels.resize(N);
      for(auto &m: runningModels)
      {
        if(!m.get())
          m = boost::make_shared<Model>();
      }
    }
    return {};
  }
};

#endif
