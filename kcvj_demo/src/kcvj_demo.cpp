#define KCVJ_GUI 1
#define KCVJ_REMOTE 1
#define KCVJ_PROFILING 1

#include "kcvj_demo.h"

void OutputHeader(ofstream & o) {
    o << "t,frame,detected,x,y,z,t_grab,";
    WriteCircleMarkerProfileHeader(o);
    o << endl;
}

int o_frame = 0;
int o_detected = 0;
double o_x, o_y, o_z = 0;
int t_grab = 0;
int64 time_start;

ofstream of;

void OutputLine(ofstream & o) {
    int o_t = (int)(GetTimeMs64() - time_start);
    o << o_t << "," << o_frame << "," << o_detected << "," << o_x << "," << o_y << "," << o_z << "," << t_grab << ",";
    WriteCircleMarkerProfileLine(o);
    o << endl;
}


int run() {
    // TODO: http://docs.opencv.org/modules/video/doc/motion_analysis_and_object_tracking.html#kalmanfilter-kalmanfilter
    //       http://opencvexamples.blogspot.com/2014/01/kalman-filter-implementation-tracking.html
    //KalmanFilter KF_ROT(?, 3, ?0);
    //KalmanFilter KF_POS(?, 3, ?0);
    
    of.open(_outFile.c_str());
    OutputHeader(of);
    
    VideoCapture * cap;
    
    int frame = _frameStart;
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, _markerSize));
    
    Camera camera(_calibrationFile);
    if (camera.cameraMatrix.empty()) return -1;
    
    Mat input_img;
    bool autoproceed = true;
    
    if (_sourceType == "capture_video" || _sourceType == "capture_camera") {
        if (_sourceType == "capture_video"  ) {
            cap = new VideoCapture(_sourceName);
        } else {
            cap = new VideoCapture(atoi(_sourceName.c_str()));
        }
        
        if (!cap->isOpened()) {
            cout << "Failed to open capture: " << _sourceName << endl;
            return -1;
        }

        if (_manualCaptureSettings) {
            cap->set(CV_CAP_PROP_FRAME_WIDTH,  _captureWidth);
            cap->set(CV_CAP_PROP_FRAME_HEIGHT, _captureHeight);
            cap->set(CV_CAP_PROP_FPS,          _captureFPS);
            cout << "Using manual capture settings; " << _captureWidth << "x" << _captureHeight << ":" << _captureFPS << endl;
        } else {
            cap->set(CV_CAP_PROP_FRAME_WIDTH,  camera.width);
            cap->set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
            cout << "Using calibration capture settings; " << camera.width << "x" << camera.height << endl;
        }
    }
    
    _translation = "/";
    time_start = GetTimeMs64();
    
    while (true) {
        if (frame > _frameStop && _frameStop != 0) frame = _frameStart;
        
        int64 t0 = GetTimeMs64();
        if (_sourceType == "image_sequence") {
            stringstream ss;
            ss << _prefix << frame << _suffix;
            cout << "Frame " << frame << endl;
            input_img = imread(ss.str(), CV_LOAD_IMAGE_COLOR);
        } else if (_sourceType == "image") {
            input_img = imread(_sourceName, CV_LOAD_IMAGE_COLOR);
        } else if (_sourceType == "capture_video" || _sourceType == "capture_camera") {
            cap->read(input_img);
        }
        
        if (input_img.empty()) {
            cout << "Image empty" << endl;
            return -1;
        }

        t_grab = (int)(GetTimeMs64() - t0);

        
        CircleMarker::findAndEstimate(input_img, output, _debug, camera, markers, _searchScale, _threshold);

        o_detected = (markers.at(0).detected ? 1 : 0);
        o_frame = frame;
        o_x = markers.at(0).t.at<double>(0);
        o_y = markers.at(0).t.at<double>(1);
        o_z = markers.at(0).t.at<double>(2);
        
        OutputLine(of);

        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected) {
                cout << markers.at(m).serialize() << endl;
                markers.at(m).detected = false;
                _translation = markers.at(m).serialize();
            } else {
                //cout << " - "  << endl;
            }
        }

        if (_debug) {
            if (!ctrl->imageSet) {
                reduceImage(output, reduced, _scale);
                ctrl->setDebugImage(reduced);
            } else if (ctrl->image_requested == 1) {
                reduceImage(output, reduced, _scale);
                pthread_mutex_lock(&(ctrl->mutex));
                ctrl->image_requested = 2;
                pthread_mutex_unlock(&(ctrl->mutex));
            }
        }
        
        #if (KCVJ_GUI == 1)
        imshow("Output", output);
        do {
            char k = waitKey(10);
            if (k == ' ') autoproceed = !autoproceed;
            if (k == 'q') return 0;
            if (k == 'b') {
                frame--;
                autoproceed = false;
                break;
            }
            if (k == 'n') {
                frame++;
                autoproceed = false;
                break;
            }
        } while (!autoproceed);
        #else
        if (frame >= _frameStop && _frameStop != 0) return 1;
        #endif
        if (autoproceed) frame++;
    }
    
    return 0;
}

int main(int argc, char* argv[]) {
    
    if (argc == 1) {
        cout << "Using default port (" << port << ") and settings file (";
        cout <<  settingsFile << ")" << endl;
    } else if (argc == 2) {
       port = atoi(argv[1]);
    } else if (argc == 3) {
       port = atoi(argv[1]);
       settingsFile = argv[2];
    } else {
       cout << "Usage: " << argv[0] << " [port] [settingsFile]" << endl;
       return -1;
    }
    
    ctrl = new RemoteControl(port, settingsFile);
    
    ctrl->settings->add("translation", &paramTranslation, false);

    ctrl->settings->add("sourceType", &paramSourceType, true);
    ctrl->settings->add("sourceName", &paramSourceName, true);
    ctrl->settings->add("searchScale", &paramSearchScale, true);
    ctrl->settings->add("markerSize", &paramMarkerSize, true);
    ctrl->settings->add("calibrationFile", &paramCalibrationFile, true);
    ctrl->settings->add("outFile", &paramOutFile, true);
    
    ctrl->settings->add("manualCaptureSettings", &paramManualCaptureSettings, true);
    ctrl->settings->add("captureHeight", &paramCaptureHeight, true);
    ctrl->settings->add("captureWidth", &paramCaptureWidth, true);
    ctrl->settings->add("captureFPS", &paramCaptureFPS, true);
    
    ctrl->settings->add("frameStart", &paramFrameStart, true);
    ctrl->settings->add("frameStop", &paramFrameStop, true);
    ctrl->settings->add("prefix", &paramPrefix, true);
    ctrl->settings->add("suffix", &paramSuffix, true);
    ctrl->settings->add("debug", &paramDebug, true);
    
    ctrl->settings->add("flip", &paramFlip, true);
    ctrl->settings->add("scale", &paramScale, true);
    ctrl->settings->add("threshold", &paramThreshold, true);
    
    if (!ctrl->settings->load(settingsFile)) {
        cout << "Couldn't find calibration file, writing defaults to " << settingsFile << endl;
        ctrl->settings->save(settingsFile);
    }
    
    bool result = run();
    ctrl->settings->save(settingsFile);
    of.close();
    return result;
}


////// //// /// / / ////////// / // //// // // ///
// Getter/Setter for setting/remote control     //
// ///// //////// //////// //////// // // / /// //

string paramFrameStart(int action, string val) {
    if (action == PARAM_SET) {
        _frameStart = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _frameStart;
        return buf.str();
    }
    return "";
}

string paramFrameStop(int action, string val) {
    if (action == PARAM_SET) {
        _frameStop = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _frameStop;
        return buf.str();
    }
    return "";
}

string paramPrefix(int action, string val) {
    if (action == PARAM_SET) {
        _prefix = val;
    } else {
        return _prefix;
    }
    return "";
}

string paramSuffix(int action, string val) {
    if (action == PARAM_SET) {
        _suffix = val;
    } else {
        return _suffix;
    }
    return "";
}

string paramSourceType(int action, string val) {
    if (action == PARAM_SET) {
        _sourceType = val;
    } else {
        return _sourceType;
    }
    return "";
}

string paramSourceName(int action, string val) {
    if (action == PARAM_SET) {
        _sourceName = val;
    } else {
        return _sourceName;
    }
    return "";
}

string paramSearchScale(int action, string val) {
    if (action == PARAM_SET) {
        _searchScale = (double) min(max(0.05, atof(val.c_str())), 1.0);
    } else {
        ostringstream buf;
        buf << _searchScale;
        return buf.str();
    }
    return "";
}

string paramMarkerSize(int action, string val) {
    if (action == PARAM_SET) {
        _markerSize = (double) min(max(0.01, atof(val.c_str())), 500.0);
    } else {
        ostringstream buf;
        buf << _markerSize;
        return buf.str();
    }
    return "";
}

string paramDebug(int action, string val) {
    if (action == PARAM_SET) {
        _debug = atoi(val.c_str()) == 1;
    } else {
        return _debug ? "1" : "0";
    }
    return "";
}


string paramManualCaptureSettings(int action, string val) {
    if (action == PARAM_SET) {
        _manualCaptureSettings = atoi(val.c_str()) == 1;
    } else {
        return _manualCaptureSettings ? "1" : "0";
    }
    return "";
}

 
string paramFlip(int action, string val) {
    if (action == PARAM_SET) {
        _flip = atoi(val.c_str()) == 1;
    } else {
        return _flip ? "1" : "0";
    }
    return "";
}

string paramCaptureHeight(int action, string val) {
    if (action == PARAM_SET) {
        _captureHeight = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _captureHeight;
        return buf.str();
    }
    return "";
}

string paramCaptureWidth(int action, string val) {
    if (action == PARAM_SET) {
        _captureWidth = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _captureWidth;
        return buf.str();
    }
    return "";
}

string paramCaptureFPS(int action, string val) {
    if (action == PARAM_SET) {
        _captureFPS = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _captureFPS;
        return buf.str();
    }
    return "";
}

string paramScale(int action, string val) {
    if (action == PARAM_SET) {
        _scale = (double) min(max(0.05, atof(val.c_str())), 1.0);
        reduceImage(output, reduced, _scale);
        ctrl->setDebugImage(reduced);
    } else {
        ostringstream buf;
        buf << _scale;
        return buf.str();
    }
    return "";
}

string paramOutFile(int action, string val) {
    if (action == PARAM_SET) {
        _outFile = val;
    } else {
        return _outFile;
    }
    return "";
}

string paramTranslation(int action, string val) {
    if (action == PARAM_SET) {
    } else {
        return _translation;
    }
    return "";
}

string paramCalibrationFile(int action, string val) {
    if (action == PARAM_SET) {
        _calibrationFile = val;
    } else {
        return _calibrationFile;
    }
    return "";
}

string paramThreshold(int action, string val) {
    if (action == PARAM_SET) {
        _threshold = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _threshold;
        return buf.str();
    }
    return "";
}

void reduceImage(Mat &src, Mat &dst, float scale) {
    if (src.empty()) return;
    Mat tmp;
    cvtColor(src, tmp, CV_RGB2GRAY);
    if (_flip)
        flip(tmp, tmp, 1);
    resize(tmp, dst, Size(), scale, scale, INTER_NEAREST);
}
