#ifndef MOBRO_UTILS_H
#define MOBRO_UTILS_H

//#define USE_CROCODDYL // otherwise use GA

#include <Eigen/Core>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;

inline int microseconds_since(const Clock::time_point &start)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now()-start).count();
}

using Eigen::Vector2d;
using Eigen::VectorXd;


#endif
