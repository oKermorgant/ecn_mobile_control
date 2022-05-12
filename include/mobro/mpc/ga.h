#ifndef GA_H
#define GA_H

#include <vector>
#include <thread>
#include <functional>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <mobro/utils.h>

namespace {

template <typename T>
inline T max(const T& o1, const T& o2)
{
  return o1 < o2 ? o2 : o1;
}
template <typename T>
inline T min(const T& o1, const T& o2)
{
  return o1 < o2 ? o1 : o2;
}

static int fastrand_seed = 0;
constexpr static float rand_denum(1.f/32767);

inline int fastrand()
{
  fastrand_seed = (214013*fastrand_seed+2531011);
  return (fastrand_seed>>16)&0x7FFF;
}

inline int fastrand(int _max)
{
  return min(_max, int(fastrand()*rand_denum * (_max+1)));
}

inline int fastrand(int _min, int _max)
{
  if(_min == _max)
    return _min;
  return _min + fastrand(_max-_min);
}

inline float fastrandf(float _min=0, float _max=1)
{
  if(_min == _max)
    return _min;
  return _min + fastrand()*rand_denum*(_max-_min);
}

inline void different_randoms(int _max, int &n1, int &n2)
{
  n1 = fastrand(_max);
  n2 = fastrand(_max-1);
  if(n1 == n2)
    n2 += 1;
}
}

// perform a single run with a random population
template <class T, uint pop_size, uint keep_best, uint max_iter, uint max_seq>
struct GA
{
  static constexpr uint half_pop = pop_size/2;
  T population[pop_size];
  T selected[half_pop-keep_best];
public:
  GA() {}

  template <class Problem>
  bool solveMulti(T& best, const Problem &prob, int max_ms)
  {
    best.cost = std::numeric_limits<double>::max();
    int comp_time(1);
    int c(0);
    T solution;
    bool better_than_best(false);
    const auto start = std::chrono::system_clock::now();
    max_ms *= 1000;
    int total(microseconds_since(start));
    while(total + comp_time < max_ms)
    {
      solve(solution, prob, max_ms - total);
      if(solution.cost < best.cost)
      {
        better_than_best = true;
        best = solution;
      }
      int dt(microseconds_since(start) - total);
      total += dt;
      comp_time = max(comp_time, dt);
      c++;
    }
    //std::cerr << "Could run " << c << " GA's in " << total/1000.f << " ms\n";
    return better_than_best;
  }


  template <class Problem>
  void solve(T &best, const Problem &prob, int max_us = 1000000)
  {
    const auto  start = std::chrono::system_clock::now();
    for(size_t i = 0; i < pop_size; ++i)
      population[i].randomize(prob);

    std::nth_element(population, population+keep_best,
                     population+pop_size);
    best = *std::min_element(population, population+keep_best);

    // loop until exit conditions
    uint iter=0, seq=0;
    float best_cost(0);

    while(microseconds_since(start) < max_us
          && iter++ < max_iter
          && seq < max_seq)
    {
      int n1, n2;
      for(int i = 0; i < half_pop-keep_best; ++i)
      {
        // do not select elite again
        different_randoms(pop_size-keep_best-1, n1, n2);
        selected[i] = min(population[n1+keep_best], population[n2+keep_best]);
      }
      for(int i = 0; i < half_pop-keep_best; ++i)
        population[keep_best+i] = selected[i];

      for(uint i=half_pop;i<pop_size;++i)
      {
        different_randoms(half_pop-1, n1, n2);

        // cross between parents + compute cost
        population[i].crossAndMutate(population[n1],population[n2], prob);
      }

      // re-sort from new costs
      std::nth_element(population, population+keep_best,
                       population+pop_size);
      best = *std::min_element(population, population+keep_best);

      if(best_cost == best.cost)
        seq++;
      else
      {
        best_cost = best.cost;
        seq = 0;
      }
    }
  }
};

#endif // GA_H