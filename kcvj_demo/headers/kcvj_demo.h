#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "RemoteControl.h"
#include "Settings.h"
#include "CircleMarker.h"


using namespace cv;
using namespace std;

RemoteControl * ctrl;
int port = 9988;
const char * settingsFile = "default.yml";

// Settings
string _sourceType = "capture_camera"; // capture_camera | capture_video | image | image_sequence
string _sourceName = "0"; // int for capture device or filename for video
string _calibrationFile = "camera.yml";
string _outFile = "output.csv";

double _markerSize = 25.0;
int _frameStart = 0;
int _frameStop = 0;


bool _manualCaptureSettings = false;
int _captureHeight = 360;
int _captureWidth = 640;
int _captureFPS = 5;

string _prefix = "frame_";
string _suffix = ".png";

double _searchScale = 0.2;
bool _flip = false;
bool _debug = true;
double _scale = 0.2;

Mat input, output, reduced;

// Setting Callbacks
string paramSourceType(int action, string val);
string paramSourceName(int action, string val);
string paramMarkerSize(int action, string val);
string paramSearchScale(int action, string val);
string paramCalibrationFile(int action, string val);
string paramOutFile(int action, string val);

string paramCaptureHeight(int action, string val);
string paramCaptureWidth(int action, string val);
string paramCaptureFPS(int action, string val);
string paramManualCaptureSettings(int action, string val);

string paramFrameStart(int action, string val);
string paramFrameStop(int action, string val);
string paramPrefix(int action, string val);
string paramSuffix(int action, string val);
string paramDebug(int action, string val);

string paramScale(int action, string val);
string paramFlip(int action, string val);
string paramProfile(int action, string val);

int run();

void reduceImage(Mat &src, Mat &dst, float scale);