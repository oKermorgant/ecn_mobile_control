#ifndef MPC_H
#define MPC_H

#include <mobro/traj.h>
#include <mobro/utils.h>

template <size_t state_dim>
struct MPCProblem
{
  using State = Eigen::Matrix<double, state_dim, 1>;
  double horizon{1.};
  double dt{0.1};
  double vmax{2.};
  double wmax{3.};
  double x_cost{10.};
  double u_cost{1.};
  std::vector<Vector2d> xref;
  State state;

  inline void resetTime(double horizon, double dt)
  {
    this->horizon = horizon;
    this->dt = dt;
  }

  inline void setCosts(double x, double u)
  {
    x_cost = x;
    u_cost = u;
  }

  inline void startWith(const State &state, double vmax, double wmax)
  {
    this->state = state;
    this->vmax = vmax;
    this->wmax = wmax;
  }

  inline size_t size() const {return xref.size();}

  inline bool prepXRef(const Traj &traj, double t0)
  {
    const auto K = std::max<size_t>(1, horizon/dt);
    const auto change{K != xref.size()};

    if(change)
      xref.resize(K);

    for(size_t i = 0; i < K; ++i)
      xref[i] = traj.xy(t0+(i+1)*dt);

    return change;
  }

  inline bool prepXRef(const Vector2d &goal)
  {
    const auto K = std::max<size_t>(1, horizon/dt);
    const auto change{K != xref.size()};

    if(change)
      xref.resize(K);

    for(auto &x: xref)
      x = goal;

    return change;
  }
};

template <size_t state_dim>
struct MPC
{
  static constexpr size_t control_dim{2};
  using State = Eigen::Matrix<double, state_dim, 1>;

public:
  explicit MPC() {}

  inline void configure(double horizon, double dt, const State &start, double vmax, double wmax)
  {
    prob.resetTime(horizon, dt);
    prob.startWith(start, vmax, wmax);
  }

  inline Vector2d compute(Vector2d goal, int max_ms)
  {
    return compute_impl(max_ms, prob.prepXRef(goal));
  }

  inline Vector2d compute(const Traj &traj, double t0, int max_ms)
  {
    return compute_impl(max_ms, prob.prepXRef(traj, t0));
  }

protected:

  MPCProblem<state_dim> prob;
  virtual Vector2d compute_impl(int max_ms, bool horizon_change) = 0;

};



#endif
