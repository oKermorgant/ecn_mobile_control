#ifndef MPC_GA_H
#define MPC_GA_H

#include <mobro/mpc/mpc.h>
#include <mobro/traj.h>
#include <mobro/mpc/ga.h>

template <size_t state_dim, class Indiv>
class GeneticMPC : public MPC<state_dim>
{
  using State = Eigen::Matrix<double, state_dim, 1>;
  using MPC<state_dim>::prob;
public:
  explicit GeneticMPC()  {  }

  // main control: start to xref


protected:
  Indiv best;
  GA<Indiv, 200, 30, 100, 30> ga;

  Vector2d compute_impl(int max_ms, bool horizon_change) override
  {
    if(horizon_change)
    {
      for(auto &indiv: ga.population)
        indiv.u.resize(prob.xref.size());
    }

    ga.solveMulti(best, prob, max_ms);

    return best.u[0];
  }
};

template <size_t state_dim>
struct GACandidate
{
  using Problem = MPCProblem<state_dim>;
  double cost{0.};
  std::vector<Vector2d> u;

  inline void randomize(const Problem &prob)
  {
    for(auto &ui: u)
    {
      ui[0] = fastrandf(-2*prob.vmax, 2*prob.vmax);
      ui[1] = fastrandf(-2*prob.wmax, 2*prob.wmax);
    }
    computeCost(prob);
  }

  inline bool operator<(const GACandidate<state_dim> &other) const
  {
    return cost < other.cost;
  }

  inline void computeCost(const Problem &prob)
  {
    cost = 0;

    // ensure saturation
    for(auto &ui: u)
    {
      ui[0] = std::clamp(ui[0], -prob.vmax, prob.vmax);
      ui[1] = std::clamp(ui[1], -prob.wmax, prob.wmax);
      cost += prob.u_cost * ui.dot(ui);
    }

    auto state{prob.state};

    for(size_t i = 0; i < prob.size(); ++i)
    {
      // move according to this input
      update(state, u[i], prob.dt);

      // compute distance
      const auto dx{state(0)-prob.xref[i](0)};
      const auto dy{state(1)-prob.xref[i](1)};
      cost += prob.x_cost*(dx*dx + dy*dy);
    }
  }

  inline void crossAndMutate(const GACandidate<state_dim> &p1, const GACandidate<state_dim> &p2, const Problem &prob)
  {
    // cut commands from p1 and p2
    const auto cut{fastrand(1, u.size()-2)};
    std::copy(p1.u.begin(), p1.u.begin()+cut, u.begin());
    std::copy(p2.u.begin()+cut, p2.u.end(), u.begin()+cut);

    // mutation
    const auto dv{0.1*prob.vmax};
    const auto dw{0.1*prob.wmax};

    for(auto &ui: u)
    {
      ui[0] += fastrandf(-dv, dv);
      ui[1] += fastrandf(-dw, dw);
    }

    computeCost(prob);
  }

  virtual void update(Eigen::Matrix<double, state_dim, 1> &state, const Vector2d &ui, double dt) = 0;
};

#endif
