#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct Result {
  vector<double> x;      // x coordinate
  vector<double> y;      // y coordinate
  vector<double> delta;  // steering angle
  vector<double> a;      // acceleration (throttle)
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  int latency_dt {2};
  double delta_prev {0};
  double acc_prev {0.1};

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
