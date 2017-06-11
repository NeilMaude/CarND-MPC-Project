#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  // Added the lag_ms value here, so that the Solve function knows the actuator lag and can adjust the values fed back...
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double lag_ms);
};

#endif /* MPC_H */
