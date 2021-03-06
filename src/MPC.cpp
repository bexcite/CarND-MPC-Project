#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
//size_t N = 20;
//double dt = 0;

double ref_cte = 0.0;
double ref_epsi = 0.0;
double ref_v = 65;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;




class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  double dt;
  size_t n;
  FG_eval(Eigen::VectorXd coeffs, double dt, size_t n) { this->coeffs = coeffs; this->dt = dt; this->n = n;}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.


    size_t x_start = 0;
    size_t y_start = x_start + n;
    size_t psi_start = y_start + n;
    size_t v_start = psi_start + n;
    size_t cte_start = v_start + n;
    size_t epsi_start = cte_start + n;
    size_t delta_start = epsi_start + n;
    size_t a_start = delta_start + n - 1;


    fg[0] = 0.0;

    // Minimize time

    // The part of the cost based on the reference state.
    for (int i = 0; i < n; i++) {
      fg[0] += 1000 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += 1000 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < n - 1; i++) {
      fg[0] += 1000 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < n - 2; i++) {
      fg[0] += 1000000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 100000 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
//      AD<double> d1 = vars[delta_start + i + 1];
//      AD<double> d0 = vars[delta_start + i];
//      if ((d1 > 0 && d0 < 0) || (d1 < 0 && d0 > 0)) {
//        fg[0] += 10000 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
//      }
    }


    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    fg[1 + delta_start] = vars[delta_start];
    fg[1 + delta_start + 1] = vars[a_start];


    // The rest of the constraints
    for (int i = 0; i < n - 1; i++) {
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];


      AD<double> delta = vars[delta_start + i];
      AD<double> a = vars[a_start + i];


      AD<double> cte0diff = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0 - y0;



      AD<double> epsi0diff = psi0 - CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);



      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.

      // TODO: Setup the rest of the model constraints
      fg[2 + x_start + i] = x1 - (x0 + 1.60934 * 1000.0 * v0 * CppAD::cos(psi0) * dt / 3600.0);
      fg[2 + y_start + i] = y1 - (y0 + 1.60934 * 1000.0 * v0 * CppAD::sin(psi0) * dt / 3600.0);
      fg[2 + psi_start + i] = psi1 - (psi0 - 1000.0 * v0 * delta / (Lf * 1.60934) * dt / 3600.0);
      fg[2 + v_start + i] = v1 - (v0 + 28000.0 * a * dt / 3600.0); // (v0 + 28000.0 * a * dt / 3600.0)

      fg[2 + cte_start + i] = cte1 - (cte0diff + 1.60934 * 1000.0 * v0 * CppAD::sin(epsi0) * dt / 3600.0);
      fg[2 + epsi_start + i] = epsi1 - (epsi0diff - 1000.0 * v0 * delta / (Lf * 1.60934) * dt / 3600.0);

    }



  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double dt) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;




  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  double prev_steer = state[6];
  double prev_throttle = state[7];


  // We should be able to plan not father than 100m
  // so we are going to estimate the N step given the average dt
  double nd = 100.0 / (1.60934 * 1000.0 * v * (dt / 3600.0));
  std::cout << "ND = " << nd << std::endl;
  size_t n = nd > 20 ? 20 : size_t(nd);
  size_t x_start = 0;
  size_t y_start = x_start + n;
  size_t psi_start = y_start + n;
  size_t v_start = psi_start + n;
  size_t cte_start = v_start + n;
  size_t epsi_start = cte_start + n;
  size_t delta_start = epsi_start + n;
  size_t a_start = delta_start + n - 1;


  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = n * 6 + (n - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = n * 6 + 2;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  vars[delta_start] = prev_steer;
  vars[a_start] = prev_throttle;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // non-actuators
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // steering -25 deg, + 25 deg
  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // throttle
  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_lowerbound[delta_start] = prev_steer;
  constraints_lowerbound[delta_start+1] = prev_throttle;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  constraints_upperbound[delta_start] = prev_steer;
  constraints_upperbound[delta_start+1] = prev_throttle;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, dt, n);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.


  std::vector<double> res( n * 6 + (n - 1) * 2 + 1);
  res[0] = n;
  for (int i = 0; i < n; ++i) {
    res[1 + i + x_start] = solution.x[x_start + i];
    res[1 + i + y_start] = solution.x[y_start + i];
    res[1 + i + psi_start] = solution.x[psi_start + i];
    res[1 + i + v_start] = solution.x[v_start + i];
    res[1 + i + cte_start] = solution.x[cte_start + i];
    res[1 + i + epsi_start] = solution.x[epsi_start + i];
    if (i < n - 1) {
      res[1 + i + delta_start] = solution.x[delta_start + i];
      res[1 + i + a_start] = solution.x[a_start + i];
    }
  }

  return res;


}
