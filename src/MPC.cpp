#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// DONE: Set the timestep length and duration
size_t N = 10; 		// Can tune this, but 10 steps at 0.1s per step is one second of look ahead
double dt = 0.1; 	// That's a reasonable balance between enough look ahead to drive well, against keeping computation time down

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

// Desired objectives
double ref_cte = 0;			// ref_cte=0 means we would like to have zero cross-track error, compared with the waypoints path
double ref_epsi = 0;		// ref_epsi=0 means we would like to have zero directional error, compared with the waypoints path
double ref_v = 40 * 0.44704;	// reference velocity - prior to setting up to seek a velocity proportional to the curvature of the waypoints path,
								// this was the desired speed
								// in the proportional version, ref_v is just used as an initial velocity target, before the proportional calc takes over
								// The multiplication is a conversion to m/s

// Weights - can tune all of these values to change the MPC behaviour
double weight_cte = 2.0;		// weighting of cross-track error
double weight_epsi = 10.0;		// weighting of psi error
double weight_v = 1.0 ;			// weighting of velocity
double weight_delta = 250.0;	// weighting of steering delta - this ends up being high, as this is important to the MPC performance on the lake track
double weight_a = 10;			// weighting of acceleration
double weight_deltadot = 30.0; // weighting of steering delta rate of change
double weight_adot = 1.0;		// weighting of acceleration rate of change

// Offsets in the vars vector - just used to make it easy to locate values in that array/vector
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// curvature estimate parameters - used to tune how the target speed is calculated
double min_curve = 100.0;		// just use this to hold the minimum curvature found so far - was useful to echo to std::cout, to see what's happening
double max_curve = 0.0;			// similar use for maximum curvature found so far
double min_prop_v = 30 * 0.44704;		// don't go below this speed when picking a proportional speed - actually select a speed from this min/max range
double max_prop_v = 50 * 0.44704;		// don't go above this speed when picking a proportional speed
double max_allow_curve = 40;	// max curve allowed before hitting the brakes to slow the car

class FG_eval {

private:
	// Evaluate a polynomial - simply borrowed from main.cpp
	double polyeval(Eigen::VectorXd coeffs, double x) {
	  double result = 0.0;
	  for (int i = 0; i < coeffs.size(); i++) {
	    result += coeffs[i] * pow(x, i);
	  }
	  return result;
	}

public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // DONE: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

	//std::cout << "Setting cost..." << std::endl;
	//std::cout << "vars.size() = " << vars.size() << std::endl;

	std::cout << "Coeffs size: " << coeffs.size() << std::endl;
	for (int i =0; i < coeffs.size(); i++) {
		std::cout << coeffs[i] << " ";
	}
	std::cout << "" << std::endl;
	// Assuming 2.5 as the polyinc and 25 poly points, get the difference between the first and last polynominal points
	// Then use this to make a rough estimate of the curvature of the desired path
	// Won't be accurate if there's a double-curve in the path, but in most cases the curve is fairly simple/quadratic
	double const poly_inc = 2.5;
	double const poly_points = 25;								// using the same increment and number of points as in the main.cpp display code -
																// so easy to relate to what's appearing in the simulator
	double near_y = polyeval(coeffs, poly_inc);					// y co-ord of nearest point
	double far_y = polyeval(coeffs, poly_inc * poly_points);	// y co-ord of furthest point
	double curve_est = std::abs(far_y - near_y);				// difference between those two points, as a rough estimate of the curvature of the path

	// Save the min/max, for display - not used in the calculation, but useful to echo to std::cout when tuning the proportional speed calc
	if (curve_est < min_curve) {
		min_curve = curve_est;
	}
	if (curve_est > max_curve) {
		max_curve = curve_est;
	}

	// Calculate a Ref_v based on the curvature of the desired track
	double prop_v = ref_v;
	double curve = 0.0;
	if (curve_est > max_allow_curve) {
		// penalise high curvature values - will mean that the reference speed drops to the min_prop_v value, saving the car from spearing off on sharp corners
		curve = max_allow_curve;
	}
	else {
		curve = curve_est;
	}
	if (prop_v > min_prop_v) {
		// the velocity is set at a level above the minimum threshold, so use the proportional velocity setting - won't need to bother at less than this speed
		// otherwise pick a reference velocity suitable for the upcoming curve
		prop_v = (max_prop_v - min_prop_v) * (max_allow_curve - curve) / max_allow_curve;		// pick a proportion of the min-to-max speed range
		prop_v += min_prop_v;																	// add on the min speed, to put this in the right range
	}

	// Echo the curvature stuff to std::cout - just helps with debugging and tuning...
	std::cout << "\nCurvature estimate: " << curve_est << ", speed set to: " << prop_v << ", min curve=" << min_curve << ", max curve=" << max_curve << "\n" << std::endl;

	fg[0] = 0;			// This is the cost function

	// Reference state cost
	// Define the cost related to the reference state (and any other constraints which will help the car drive!)
	for (int i = 0; i < N; i++)
	{
		fg[0] += weight_cte * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
		fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
		fg[0] += weight_v * CppAD::pow(vars[v_start + i] - prop_v, 2);				// replaced ref_v with the prop_v calculated above
	}
	for (int i = 0; i < N - 1; i++)
	{
		fg[0] += weight_delta * CppAD::pow(vars[delta_start + i], 2);
		fg[0] += weight_a * CppAD::pow(vars[a_start + i], 2);
	}
	for (int i = 0; i < N - 2; i++)
	{
		fg[0] += weight_deltadot * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
		fg[0] += weight_adot  * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
	}

	// Initial constraints, noting that fg[0] is the cost and so all of these are bumped along by one in the fg[] vector
	fg[1 + x_start] = vars[x_start];
	fg[1 + y_start] = vars[y_start];
	fg[1 + psi_start] = vars[psi_start];
	fg[1 + v_start] = vars[v_start];
	fg[1 + cte_start] = vars[cte_start];
	fg[1 + epsi_start] = vars[epsi_start];

	// The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      // Use the polynominal coefficients here, using all coefficients to calc initial values
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;		// changed from given class calc - can also use pow() here
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);		// changed from given class calc - can also use pow() here

      // Here's `x` to get you started.
      // The idea here is to constrain this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double lag_ms) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // DONE: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;			// 6 components of state: x,y,psi,v,cte,epsi
  // DONE: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // DONE: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  // This represents the maximum steering angle range for the car
  for (int i = delta_start; i < a_start; i++) {
	vars_lowerbound[i] = -0.436332;					// could use the deg3rad() function here, but easy to just use constants
	vars_upperbound[i] = 0.436332;
  }

  // Acceleration/deceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  // Represents maximum acceleration and deceleration of the car - -1.0 to 1.0 seems to be the max range, setting wider doesn't seem to make a difference...
  for (int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
	    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
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

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  options += "Numeric max_cpu_time          0.5\n";			// after this period, the solver will return whatever solution it has - consider the N value...

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

  // DONE: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> result;

  // account for lag, by moving up the solution path - i.e. use the steering angle and throttle from one of the future time-step states of the solution
  int index_shift = std::min(int(std::round(lag_ms/(1000 * dt))), int(N-1));
  std::cout << "Accounting for lag of: " << lag_ms << " by looking ahead " << index_shift << " steps" << std::endl;

  result.push_back(solution.x[delta_start + index_shift]);			// actuations to use for the next timestep
  result.push_back(solution.x[a_start + index_shift]);

  for (int i = 0; i < N; i++)
  {
	  result.push_back(solution.x[x_start + i + 1]);	// future MPC locations (used to draw the green line)
	  result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
