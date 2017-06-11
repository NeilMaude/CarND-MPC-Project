# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Overview
This project is an implementation of an MPC (model predictive control) controller for use with the Udacity Self-Driving Car Simulator.

The MPC implementation attempts to drive a car around the "lake track" circuit, choosing steering and accelerator inputs by fitting an optimised projected path to a desired reference path.

## Model
The model contains 6 state values as follows:
* x - x co-ordinate of the car
* y - y co-ordinate of the car
* psi - orientation of the car
* v - velocity of the car
* cte - cross-track error of the car
* epsi - orientation error of the car

These state parameters are inputs to the MPC calculations, as per the `MPC::Solve()` function in `MPC.cpp`.

The MPC calculation attempts to determine the following:
* delta - the best steering angle at the current time-step (after allowing for latency)
* a - the best throttle setting at the current time-step (after allowing for latency)

These values are determined by the solver, which uses the bicycle motional model to determine settings which create a path which best fits to the polynominal describing the desired path.  The cost function for this optimisation problem attempts to minimise the total cost of a variety of weighted inputs (see `MPC::FG_eval` and below).  

## Timestep Choices
The MPC has parameters for how many time steps to look forward, `N`, and the duration of each time step, `dt`.  The class examples given were `N=25` and `dt=0.05`.  However, large values of N make the computation time-consuming and from the outset a lower value (`N=10`) was chosen.  Considering the need to consider the immediate path of the car, `dt=0.1` was chosen, giving a look-ahead of 1 second.  These parameters were chosen largely on back-of-an-envelope reckoning, but appear to work well in practice.

## Implementation Approach
The initial approach was to implement a basic MPC controller, using fixed reference parameters for cte (cross track error), psi (direction) error and velocity.  In particular, for the initial implementation the velocity was set to a fixed value.  This means that the MPC would attempt to drive the car around the track at this fixed velocity.

This was first done with zero latecy/lag, implementing the MPC to achieve a lap of the track. Once this was done, compensation for the lag was added.

The MPC controller parameters were then manually tuned to optimise the performance, allowing a gradual increase in the fixed velocity.

Finally, the MPC controller cost function calculation was updated to use a target/reference velocity setting proportional to the curvature of the desired path.  This allows faster speeds to be set on straight sections of the track and a reduced speed for corners.  

## Fixed Velocity Implementation
The MPC code is as implemented in `MPC.cpp`.  

### Latency Consideration
Consideration of the actuator lag/latency is handled by passing the lag time (in ms) to the MPC solver.  The solver then uses the lag time to pick a time-step from which to select steering/throttle settings.  This choice of settings from a future time-step means that the actuator parameters passed to the car are (approximately) correct for the time at which those settings will actually be used. (Note that in a practical implementation the latency may well vary and this may need some additional compensation, such as calculating the time taken for the actual solution calculation - for simplicity, the only lag taken into account was the fixed latency set in the project.)

### Parameter Tuning
Considerable parameter tuning was done, to get to a stable high-speed solution. 

"Tunable" parameters are:
* CTE weight (weighting of cross-track error)
* EPSI weight (weighting of directional error)
* V weight (weighting of velocity error)
* Delta weight (weighting of steering angle error)
* A weight (weighting of acceleration error)
* DeltaDot weight (weighting of error in rate of steering angle change)
* ADot weight (weighting of error in rate of acceleration change)

Each weighting determines how much a variation in the given parameter (from the ideal path) contributes to the cost function being minimised to find the selected path.

Initial best-guesses and iterative testing gave settings of:
* w_cte = 1.0
* w_epsi = 10.0
* w_v = 1.0
* w_delta = 500.0
* w_a = 10.0
* w_deltadot = 30.0
* w_adot = 1.0

These settings allowed a lap at reference velocities up to 60mph.

The following refinements were then made to the settings (variations listed, all other parameters retained the same):
* w_cte=0.5 - allows lapping at 70mph reference velocity
* w_delta = 3000.0 - allows lapping at 80mph reference velocity

The results of the fixed velocity implementation can be seen at: [80mph fixed velocity](https://youtu.be/pF3_dLk3f70).

Further refinements showed diminishing returns, with the final variation being:
* w_delta = 4000.0, w_deltadot = 100.00 - allows lapping at 90mph reference velocity

The results of this can be seen at: [90mph fixed velocity](https://youtu.be/_ZTIyGT9XIo).

Beyond this it did not appear possible to find settings which allowed the car to navigate the sharper corners at higher speeds.

After refining to allow lapping at 90mph, the reference velocity was set back to 50mph but all other parameters left the same.  This was to prove that the solution is stable.  The car then lapped for extended periods at 50mph - likely indefintely. A short example of this can be see at: [50mph fixed velocity](https://youtu.be/2jaa8UX3CvY).

It was noted that - especially with the fixed velocity target - the MPC was mainly concerned with the variation of steering angle from the ideal path and setting a high weighting to this value gave the best results.

## Variable Reference Velocity
Logically, it makes sense for the car to travel faster on straights than around corners - this is especially the case when latency is introduced. 

(Note: with zero latency it would be possible to go faster around corners without needing to slow down, because the physics engine in the simulator does not appear to consider wheel slippage - the only constraint would be around steering angle maximums).

To allow for faster speeds on the straight sections, the code was modified to include a simple estimate of the curvature of the desired path (magnitude of variation in the y co-ordinates from the start to end of the desired path) and the size of this estimate was used to set the reference (target) velocity at the start of each path calculation.  

This introduced two new parameters for tuning:
* minimum velocity - the minimum speed for the car
* maximum curve - the maximum curvature estimate allowed for the proportional calculation

These parameters are really more like contraints on the proportional velocity calculation, as the minimum velocity specifies a lower bound for the reference velocity (using the previous parameter tuning, we know that the car can go around the track at least this fast) and the maximum curve specifies a bound above which the reference speed will always be set to the minimum (i.e. once curvature reaches a certain level, the target speed is always reduced to the minimum level). 

An example of the MPC performance with proportional reference velocity control can be seen at: [100mph straight, variable speed](https://youtu.be/meqTQ0j41Eg).  This example uses a minimum velocity setting of 80mph and maximum curvature setting of 30.0.

Further tuning was done by setting the maximum curvature allowed to 40.0, which provided a (reasonably!) safe lap with a maximum observed speed of 103mph.  This can be seen here: [103mph, variable speed](https://youtu.be/q0VCIWiL32s).

The above is the current state of the project, as checked-in to this repo.

(Note that at certain times - especially at the end of the right-left section on the lake-shore - the car is taking a fairly wide line.  However, this is at extreme speeds!)

Further tuning may allow even faster speeds, potentially by addressing some of the over-braking for some corners - the car is dropping below the previous optimised corner speed from the 90mph fixed-speed lap.  

## Future Enhancements
An obvious enhancement to this project would be to implement an INI file mechanism to store the multitude of parameters.  This would remove the painful process of re-compiling the project for each test run.

Secondly, the parameter tuning to date has been manual.  This could be improved with a "twiddle" style approach, which tunes the parameters automatically, possibly using the maximum observed CTE as a measure of the cost of each solution (certainly seeking solutions where the car remains on the track surface throughout).

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


