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
string _sourceType = "capture"; // capture | image
string _sourceName = "0"; // int for capture device or filename for video
string _calibrationFile = "camera.yml";
int _frameStart = 0;
int _frameStop = 0;
string _prefix = "frame_";
string _suffix = ".png";

double _searchScale = 0.2;
bool _flip = true;
double _scale = 0.2;

Mat input, output, reduced;

// Setting Callbacks
string paramSourceType(int action, string val);
string paramSourceName(int action, string val);
string paramSearchScale(int action, string val);
string paramCalibrationFile(int action, string val);

string paramFrameStart(int action, string val);
string paramFrameStop(int action, string val);
string paramPrefix(int action, string val);
string paramSuffix(int action, string val);

string paramScale(int action, string val);
string paramFlip(int action, string val);
string paramProfile(int action, string val);

int run();

void reduceImage(Mat &src, Mat &dst, float scale);