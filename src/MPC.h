#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <ctime>

using namespace std;

const double ACC_MAX = 2.8;
const double ACC_MIN = -2.8;
const double MAX_SPEED = 85.0; // the ideal max speed
// reference velocity
const double ref_v = MAX_SPEED * 0.44704;

const size_t POLY_ORDER = 3;

// N / dt / Lf
// 12/ .10/2.65/45 - Good
// 12/ .15/2.65/76 - better up to 76 speed
// 11/ .14/2.65/77 - better up to 77 speed
//  9/ .15/3.25/77 - even better up to 77 speed
//  9/ .13/3.55/87 - speed 87
//  9/ .13/3.30/87 - Good under steer but over corrects on sharp curves
// 10/ .13/3.30/87 - worst then above
//  8/ .13/3.30/87 - better
//  9/ .13/3.10/77 - equal over and under correction
//  9  .13/3.10/80 - still within the lines at higher speed.  Reduce N
//  8/ .13/3.10/80 - better than 9/.13/3.10/80 due to lest over correction
//  8/ .13/3.10/85
const size_t N = 8;
const double dt = 0.13;

// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 3.30; // 2.65



class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  void reload();

  vector<double> x_vals;
  vector<double> y_vals;
  double last_steering;
};

#endif /* MPC_H */
