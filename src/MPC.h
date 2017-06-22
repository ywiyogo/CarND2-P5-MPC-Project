#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct MpcSolution{
  vector<double> xpts;
  vector<double> ypts;
  vector<double> cte;
  vector<double> epsi;
  double delta;
  double a;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  MpcSolution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
};

#endif /* MPC_H */
