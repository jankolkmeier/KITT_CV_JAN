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
For GNU/Linux distributions, just install the version from the official repositiories (ex. libopencv-dev, for ubuntu). In any
case, it is importand that after installation, [pkg-config](http://en.wikipedia.org/wiki/Pkg-config)
knows of OpenCV. To test this, check if:
  `pkg-config --cflags opencv` 

returns the path of where OpenCV headers have been installed.

For one of the example applications, we use nonfree parts of OpenCV, which are not shipped with all OpenCV distributions.
On Ubuntu, these can be added with the following commands:
  `sudo add-apt-repository --yes ppa:xqms/opencv-nonfree`
  `sudo apt-get update` 
  `sudo apt-get install libopencv-nonfree-dev`


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

Run cmake in the build directory with the respective VS version (here for VS (Express) 2012):
  `cd ./build`
  `cmake .. -G "Visual Studio 11 Win64" -DCMAKE_PREFIX_PATH=C:\opencv\build`

If it is a 32bit system, leave out the `Win64` part. Change the additional prefix paramenter to the location of
your OpenCV installation (the one with "OpenCVConfig.cmake" inside).

Open the VS solution file (`KITT_CV_JAN.sln`) inside the build directory. You should be able to build it right away.


### First Run
It is recommended to first run the "minimal example" (kcvj_minimal).

Note that each IDE (XCode, VS) has different ways of where to compile the binaries to and in what directory
to run them. If you use make on Linux, all binaries will be put inside the ./build directory. If you use
Visual Studio, binaries are placed in a "Debug" or "Release" foldner, however, the running directory when
debugging from VS is the "build" directory. XCode also has a Debug directory for the binaries, and uses
that directory as running directory when debugging.

This is important to know when trying to load files such as XML's for camera calibration.

For the minimal example, compile it, and put the "logitech800x600.yml" file inside the respective running directory.
Note that it is recommended to make your own camera calibration file suited for the camera you are using.
Instructions for performing camera calibration can be found further below.

## Examples

In the following we describe the various examples included in the project.

### Demo (./kcvj_demo)
The demo applications is the most extensive application in the project. It adds extra functionality
around the KCVJ library:
 
 - Load/save settings to/from files
 - Remote control to send data and receive commands from 3rd applications
 - Inspect image sequences on a frame-by-frame bases
 - Log results

The remote function starts a seperate thread and listens on a socket. This uses unix threads and socket implementations,
and therefore is not working under windows!

### Minimal Example (./kcvj_minimal)
The minimal example contains the bare minimum needed to test the kcvj library. No command line parameters - check the source code
to adjust basic settings (calibration file, marker size, threshold, using gui, etc...).

### Multiple Cameras (./kcvj_multicam)
An example with several cameras at the same time - each computing their own position (not a common rig position) - check the source code
to adjust basic settings (calibration file, marker size, threshold, using gui, etc...) as well as the number of cameras.


## Resources

### Calibration files (./resources/calibration)
This folder contains some calibration files for webcams used during development.
The file "macbook.yml" is for the integrated webcam found in newer macbooks at native resolution.
The file "macbook640x480.yml" is for the same webcam but at reduced capture resolution.
Similarly for "logitech.yml" and "logitech800x600.yml", but for the Logitech QuickCam Pro 90000.
"edge.yml" is for the default 0.3mp cam on thinkpad edge series notebooks. "unity.xml" is the calibration
for a virtual camera used for simulations rendered in the 3D Unity game engine.

### Configuration files (./resources/configurations)
Some sample configuration files. These should be inspected befor using them to use the desired paths
and camera calibration files. 

TODO: image_sequence (test)
TODO: capture_camera

### Web Remote Control (./resources/remoteControl)
This contains a web-based control and debugging application that runs together with the kcvj_demo application.
It needs nodejs installed to run. It does not ship with third-party plugins, so running

`npm install png engine.io node-static`

First is required to load the plugins.
Then, run it with
`nodejs RemoteControlWebClient.js`

It can be run either on the same computer as kcvj_demo runs, or on a remote computer.
This requires to uncomment and change the last lines of "RemoteControlWebClient.js" to use ip and port 
of the computer that runs kcvj_demo. The web interface is hosted on port :8080.

### Tests (./resources/tests)
The tests folder contains some image files used by example applications for testing purposes.
Most importantly, the files in ./resources/tests/optitrack, which also come with a CSV file 
containing ground truths for each frame (the CSV is in "./resources/stats/optitrack_truth.csv").

### Statistics (./resources/stats)
This folder contains some matlab ".m" scripts and recorded ".csv" logs that were 
used for evaluation of the system.

## Extra Applications

### Camera Calibration (./calibration)
The camera calibration application is part of the main project and compiled with all other targets.
Running it without parameters gives some thorough help. Here are some examples as for how I have used it:

`./calibration -w 8 -h 5 -g -pt chessboard -cw 800 -ch 600 -s 0.4 -d 1500 -o philips800x600.yml -n 20 0`

We use a chessboard pattern with 8x5 corners. The "-g" flag enables gui output (not to be used on "headless" computers).
The "-cw" and "-ch" parameters define the desired maximum capture height and width (opencv will pick that one or the next smaller one
supported by the webcam). The "-s" defines a size unit for the squares (in this case 4cm). "-d" indicates the minimal delay between frames
- this gives some time to alternate the orientation of the cessboard.
The output file ("-o") is phillips800x600.yml. We take "-n" = 20 sample pictures of the chessboard in view to base our calibration on.
The last parameter is "0", which refers to the camera capture device id.

If calibration is too heavy on the desired device, and the camera cannot be attached to a regular PC, pass the "-so" parameter,
which only saves images with the desired interval ("-d"). These can then be loaded on another computer and be used for calibration.
Read the instructions on how to load image lists in the help output.

### Hough Circle Test (./hough_circle_test)
A sample to test execution performance of the hough circle detection. Otherwise not functional.

### ArUco Test (./aruco_test)
A sample to test execution performance of the ArUco library. Otherwise not functional.

### Feature Test (./feature_test)
A sample to test execution performance of different feature based detection and matching approaches. Otherwise not functional.