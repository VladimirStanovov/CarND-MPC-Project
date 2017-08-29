#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

using CppAD::AD;

typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  //CppAD::ipopt::solve_result<Dvector> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double prevDelta = 0;
  double prevA = 0;
};

struct Solution
{

};

#endif /* MPC_H */
