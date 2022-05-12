#ifndef MOBRO_MPC_CT
#define MOBRO_MPC_CT

#include <ct/core/Systems>
#include <ct/optcon/optcon.h>
#include <mobro/traj.h>
#include <visp/vpColVector.h>

using namespace ct::core;
using namespace ct::optcon;
using Clock = std::chrono::steady_clock;

template <size_t state_dim>
class MobileMPC
{
  static constexpr size_t control_dim{2};
  using ModelPtr = std::shared_ptr<ControlledSystem<state_dim, 2>>;
  using Qtype = Eigen::Matrix<double, state_dim, state_dim>;
  using Rtype = Eigen::Matrix2d;
  using Xtype = StateVector<state_dim>;
  using Utype = ControlVector<2>;

public:
  explicit MobileMPC(ModelPtr model) : model(model), adLinearizer(std::make_shared<ct::core::SystemLinearizer<state_dim, control_dim>>(model))
  {
    // solver settings
    ilqr_settings.integrator = IntegrationType::EULERCT;
    ilqr_settings.discretization = NLOptConSettings::APPROXIMATION::FORWARD_EULER;
    ilqr_settings.max_iterations = 10;
    ilqr_settings.nlocp_algorithm = NLOptConSettings::NLOCP_ALGORITHM::ILQR;
    ilqr_settings.lqocp_solver = NLOptConSettings::LQOCP_SOLVER::
                                 GNRICCATI_SOLVER;  // the LQ-problems are solved using a custom Gauss-Newton Riccati solver
    ilqr_settings.printSummary = false;

    intermediateCost = std::make_shared<ct::optcon::TermQuadratic<state_dim, control_dim>>();
    finalCost = std::make_shared<ct::optcon::TermQuadratic<state_dim, control_dim>>();

    // default settings
    forwardParam(1., 0.1);
    setR(0.1);

    costFunction = std::make_shared<CostFunctionAnalytical<state_dim, control_dim>>();
    costFunction->addIntermediateTerm(intermediateCost);
    //costFunction->addIntermediateTerm(intermediateCost);



    // MPC part
    ilqr_settings_mpc = ilqr_settings;
    ilqr_settings_mpc.max_iterations = 1;

    mpc_settings.stateForwardIntegration_ = true;
    mpc_settings.postTruncation_ = true;
    mpc_settings.measureDelay_ = true;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME;
    mpc_settings.coldStart_ = false;
  }

  inline void forwardParam(double horizon, double dt)
  {
    this->horizon = horizon;
    ilqr_settings.dt = ilqr_settings_mpc.dt = dt;
  }

  inline void setR(double r)
  {
    Qtype Q{Qtype::Zero()};

    Q(0,0) = 1.;
    Q(1,1) = 1.;
    const auto R{r*Rtype::Identity()};

    intermediateCost->setWeights(Q, R);
    finalCost->setWeights(Q, R);
  }

  inline size_t prepXRef()
  {
    xref.resize(ilqr_settings.computeK(horizon)+1);
    return xref.size();
  }

  vpColVector compute(vpColVector start, vpColVector goal, double t0)
  {
    const auto K{prepXRef()};

    // build xref from fixed goal
    for(size_t i = 0; i < K; ++i)
    {
      xref[i](0) = goal[0];
      xref[i](1) = goal[1];
    }
    return compute(start, t0);
  }

  vpColVector compute(vpColVector start, const Traj &traj, double t0)
  {
    const auto K{prepXRef()};
    for(size_t i = 0; i < K; ++i)
    {
      const auto [pd,vd,ad] = traj.ref(t0+i*ilqr_settings.dt); {}
      xref[i](0) = pd[0];
      xref[i](1) = pd[1];
    }
    return compute(start, t0);
  }

protected:
  // model-related ptrs
  ModelPtr model;
  std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer;

  // opt con variables
  NLOptConSettings ilqr_settings, ilqr_settings_mpc;
  ct::core::Time horizon{1.};
  StateVectorArray<state_dim> xref;
  std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> intermediateCost;
  std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> finalCost;
  std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> costFunction;

  // mpc variables
  ct::optcon::mpc_settings mpc_settings;

  // main control: start to xref
  vpColVector compute(vpColVector start, double t0)
  {
    const auto K{xref.size()-1};
    Xtype x0;
    for(uint i = 0; i < state_dim; i++)
      x0(i) = start[i];

    FeedbackArray<state_dim, control_dim> u0_fb(K, FeedbackMatrix<state_dim, control_dim>::Zero());
    ControlVectorArray<control_dim> u0_ff(K, ControlVector<control_dim>::Zero());
    typename NLOptConSolver<state_dim, control_dim>::Policy_t initController
        (xref, u0_ff, u0_fb, ilqr_settings.dt);


    // create (TODO configure) optConProblem
    ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        horizon, x0, model, costFunction, adLinearizer);

    // create (TODO configure) iLQR
    NLOptConSolver<state_dim, control_dim> iLQR(optConProblem, ilqr_settings);
    iLQR.setInitialGuess(initController);

    // get an initial guess for MPC
    iLQR.solve();
    ct::core::StateFeedbackController<state_dim, control_dim> initialSolution = iLQR.getSolution();

    // create (TODO configure) MPC
    MPC<NLOptConSolver<state_dim, control_dim>> ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);
    ilqr_mpc.setInitialGuess(initialSolution);

    const auto start_time{Clock::now()};
    ct::core::StateFeedbackController<state_dim, control_dim> optCon;
    for(size_t it = 0; it < 100; it++)
    {
      auto current_time = Clock::now();
      auto t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();

      ilqr_mpc.prepareIteration(t);

      // timestamp of the new optimal policy
      ct::core::Time ts_newPolicy;
      current_time = Clock::now();
      t = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time).count();
      bool success = ilqr_mpc.finishIteration(x0, t, optCon, ts_newPolicy);

      if (ilqr_mpc.timeHorizonReached() | !success)
          break;
    }

    Utype u = Utype::Zero();
    if(ilqr_mpc.timeHorizonReached())
    {
    // extract control
    optCon.computeControl(x0, 0., u);
    std::cout << "Control found: " << u.transpose() << std::endl;
    }

    return {u(0), u(1)};
  }
};

#endif