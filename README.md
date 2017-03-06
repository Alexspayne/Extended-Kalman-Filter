# Extended Kalman Filter Project

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

# Rubric Points
I explain how I met each of the specifications in the project rubric.

## Compiling
### Your code should compile.
It compiles.

## Accuracy
### The px, py output coordinates have an RMSE <= [0.8, 0.8, 0.26, 0.28] compared to the ground truth.

My RMSE was <Insert result here>.

## Follows the Correct Algorithm

### Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.


### Your Kalman Filter algorithm handles the first measurements appropriately.


  * Your algorithm should use the first measurements to initialize the state and covariance matrices.

### Your Kalman Filter algorithm first predicts then updates.


  * Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.

### Your Kalman Filter can handle radar and lidar measurements.

## Code Efficiency

### Your algorithm should avoid unnecessary calculations.

Avoid:


  * Running the exact same calculation repeatedly when you can run it once, store the value and then reuse the value later.

  * Loops that run too many times.

  * Creating unnecessarily complex data structures when simpler structures work equivalently.

  * Unnecessary control flow checks.
