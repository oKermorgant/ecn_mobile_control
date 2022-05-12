///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019, LAAS-CNRS
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "crocoddyl/core/actions/unicycle.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/utils/timer.hpp"

#include <log2plot/logger.h>

int main(int argc, char* argv[])
{
  unsigned int N = 20;  // number of nodes
  unsigned int T = 5e1;  // number of trials
  unsigned int MAXITER = 10;
  if (argc > 1)
    T = atoi(argv[1]);

  system("killall python");

  // Creating the action models and warm point for the unicycle system
  Eigen::VectorXd x0 = Eigen::Vector3d(0., 1., 0);
  auto model = boost::make_shared<crocoddyl::ActionModelUnicycle>();
  model->set_cost_weights({10., 1.});
  std::vector<Eigen::VectorXd> xs(N + 1, x0);
  std::vector<Eigen::VectorXd> us(N, Eigen::Vector2d::Zero());
  std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract> > runningModels(N, model);

  // Formulating the optimal control problem
  boost::shared_ptr<crocoddyl::ShootingProblem> problem =
      boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels, model);
  crocoddyl::SolverDDP ddp(problem);

  // Solving the optimal control problem
  log2plot::Logger logger("/home/olivier/Results/mobro/");

  auto u{us[0]};
  logger.save(x0, "x", "[x,y,\\theta]", "Pose");
  logger.save(u, "u", "[v, \\omega]", "u");

  logger.update();

  Eigen::ArrayXd duration(T);
  for (unsigned int i = 0; i < T; ++i)
  {
    problem->set_x0(x0);
    ddp.solve(xs, us, MAXITER);
    x0 = ddp.get_xs()[1];
    u = ddp.get_us()[0];
    logger.update();
  }
  logger.plot();

  return 0;

  double avrg_duration = duration.sum() / T;
  double min_duration = duration.minCoeff();
  double max_duration = duration.maxCoeff();
  std::cout << "  DDP.solve [ms]: " << avrg_duration << " (" << min_duration << "-" << max_duration << ")"
            << std::endl;

  // Running calc
  for (unsigned int i = 0; i < T; ++i) {
    crocoddyl::Timer timer;
    problem->calc(xs, us);
    duration[i] = timer.get_duration();
  }

  avrg_duration = duration.sum() / T;
  min_duration = duration.minCoeff();
  max_duration = duration.maxCoeff();
  std::cout << "  ShootingProblem.calc [ms]: " << avrg_duration << " (" << min_duration << "-" << max_duration << ")"
            << std::endl;

  // Running calcDiff
  for (unsigned int i = 0; i < T; ++i) {
    crocoddyl::Timer timer;
    problem->calcDiff(xs, us);
    duration[i] = timer.get_duration();
  }

  avrg_duration = duration.sum() / T;
  min_duration = duration.minCoeff();
  max_duration = duration.maxCoeff();
  std::cout << "  ShootingProblem.calcDiff [ms]: " << avrg_duration << " (" << min_duration << "-" << max_duration
            << ")" << std::endl;            
}
