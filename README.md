# PID Controller Project

This repository contains a C++ implementation of a PID controller for the Udacity Self-Driving Car Nanodegree Program.

## Project Structure

- `src/`: Source code for the PID controller and the main application.
  - `PID.cpp` / `PID.h`: The PID controller class.
  - `main.cpp`: The main application that interacts with the simulator via WebSocket.
- `include/`: Header files for external libraries (JSON).
- `CMakeLists.txt`: Build configuration file.
- `install-mac.sh` / `install-ubuntu.sh`: Scripts to install uWebSockets.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`.
* [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases)

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`

## PID Controller Details

A PID controller calculates an "error" value as the difference between a measured process variable and a desired setpoint. The controller attempts to minimize the error by adjusting the process control inputs.

The PID controller consists of three terms:
1.  **Proportional (P):** Accounts for present values of the error. For example, if the error is large and positive, the control output will also be large and positive.
2.  **Integral (I):** Accounts for past values of the error. If the current output is not sufficiently strong, the integral of the error will accumulate over time, and the controller will respond by applying a stronger action.
3.  **Derivative (D):** Accounts for possible future values of the error, based on its current rate of change.

### Tuning

The hyperparameters (P, I, D coefficients) in `main.cpp` were tuned manually.
-   **P (0.15):** Controls the steering response to Cross Track Error (CTE). A higher value turns the car faster towards the center but causes oscillation.
-   **I (0.0):** Corrects for systematic bias (drift). Since the simulator is ideal, this was kept at 0.0 to avoid instability.
-   **D (2.5):** Damps the oscillation caused by the P term. A higher value reduces overshoot but can make the steering jittery.

## Testing

To run the unit tests:

```bash
cd build
make test_pid
./test_pid
```
