README                 {#readme}
======

This project contains the KCVJ library which can be used to perform inside out camera tracking
with CircleMarkers as described in the internship report. The KCVJ library is documented
in [KCVJ_LIB.md](index.html). 

This documents describes installation and setup, contained resources as well as smaller
examples, and other extra - non-kcvj - applications.

## Setup

### Dependencies
The only required 3rd party library is OpenCV. Any version in 2.4.x should work, it was last
tested with 2.4.9. Guides on compiling OpenCV yourself can be found over the internet, but
using the precompiled binaries for the respective platform works fine. I used the
[OpenCV Macports packgage](https://www.macports.org/ports.php?by=library&substr=opencv) on OS X.
For GNU/Linux distributions, just install the version from the official repositiories. In any
case, it is importand that after installation, [pkg-config](http://en.wikipedia.org/wiki/Pkg-config)
knows of OpenCV. To test this, check if:
  `pkg-config --cflags opencv` 

returns the path of where OpenCV headers have been installed.

### Generate Project Files

I created [CMake](http://cmake.org) configuration files for the build. To create project files
for XCode, Visual Studio, or the makefile for gcc, first create a *build* directory inside the
*KITT_CV_JAN* directory, then follow the respective instrucitons.

##### 1 Linux / makefile

Run cmake in the build directory:
  `cd ./build`
  `cmake .. `

You can build all code by running
  `make`

##### 2 OS X / Xcode

Run cmake in the build directory:
  `cd ./build`
  `cmake .. -G Xcode`

Open the Xcode project file inside the build directory and you should be able to build it right away.

##### 3 Windows / Visual Studio

Run cmake in the build directory with the respective VS version:
  `cd ./build`
  `cmake .. -G "Visual Studio 10 Win64"`

Open the VS project file inside the build directory and you should be able to build it right away.

## Examples

### Demo (./kcvj_demo)
The demo applications is the most extensive application in the project. It adds extra functionality
around the KCVJ library:
 
 - Load/save settings to/from files
 - Remote control to send data and receive commands from 3rd applications
 - Inspect image sequences on a frame-by-frame bases
 - Log results

### Minimal Example (./kcvj_minimal)
The minimal example contains the bare minimum needed to test the kcvj library.

### Multiple Cameras (./kcvj_multicam)
An example with several cameras at the same time - each computing their own position (not a common rig position).


## Resources

### Calibration files (./resources/calibration)

### Configuration files (./resources/configurations)

### Web Remote Control (./resources/remoteControl)

### Tests (./resources/tests)

### Statistics (./resources/stats)


## Extra Applications

### Camera Calibration (./calibration)

### Hough Circle Test (./hough_circle_test)

### Aruco Test (./aruco_test)

### Feature Test (./feature_test)



CircleMarker::estimateMarkerPose

test
