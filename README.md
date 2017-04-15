# Extended Kalman Filter

My solution of CarND-Extended-Kalman-Filter-Project assignment from Udacity Self Driving Car nanodegree course, Term 2. See project assignment starter code in https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

---

## Dependencies

The project compilation and work have been verified under the following platforms: 
* Mac OX Sierra XCode 8.3.1 
* Windows 10 Visual Studio 2015 64-bit

I used cmake 3.7.2 to build project files.

## Code Style

To enforce [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html), I included Google's `cpplint.py` file available in `./src/Lint` folder. This tool expects the presense of python2.7 installed. To check code style of my code, run the following command line (from `./src/Lint`):

```
./cpplint.py ../*.h ../*.cpp
```

## Project Structure

The project consists of **ExtendedKF** and **unittests** applications.

**ExtendedKF** application has the same command line syntax as in the original [repo](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). Use the same build commands to generate and compile project files as described there.

For **unittests**, I used [Catch](https://github.com/philsquared/Catch) framework, consisting of a single file in `./src/Catch` folder. All unit tests are implemented in `./src/unittests.cpp` file.

For more expressive and laconic code, I enforced using C++11 standard in my project files by -std=c++11 command.

The code consists of the following modules:
* `FusionEKF.h`/`.cpp` implements sensor fusion code for pedestrian detection by Lidar and Radar sensors
* `kalman_filter.h`/`.cpp` implements general kalman filter math, common to linear and extended kalman filter measurement updates
* `tools.h`/`.cpp` implements nonlinear measurement calculations for Radar sensor
* `ground_truth_package.h`/`measurement_package.h` declares structures for ground truth and measurement data
* `main.cpp` is an entry point for **ExtendedKF** application
* `unittests.cpp` is an entry point for **unittests** application
