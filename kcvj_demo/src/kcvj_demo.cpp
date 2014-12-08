#define KCVJ_GUI 0
#define KCVJ_REMOTE 1

#include "kcvj_demo.h"

int run() {
    VideoCapture * cap;
    
    int frame = _frameStart;
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, _markerSize));
    
    Camera camera(_calibrationFile);
    if (camera.cameraMatrix.empty()) return -1;
    
    Mat input;
    bool autoproceed = true;
    
    if (_sourceType == "capture_video" || _sourceType == "capture_camera") {
        if (_sourceType == "capture_video" ) {
            cap = new VideoCapture(_sourceName);
        } else {
            cap = new VideoCapture(atoi(_sourceName.c_str()));
        }
        
        if (!cap->isOpened()) {
            cout << "Failed to open capture: " << _sourceName << endl;
            return -1;
        }
    }
    
    
    while (true) {
        if (frame > _frameStop) frame = _frameStart;
        
        if (_sourceType == "image_sequence") {
            stringstream ss;
            ss << _prefix << frame << _suffix;
            cout << "Frame " << frame << endl;
            input = imread(ss.str(), CV_LOAD_IMAGE_COLOR);
        } else if (_sourceType == "image") {
            input = imread(_sourceName, CV_LOAD_IMAGE_COLOR);
        } else if (_sourceType == "capture_video" || _sourceType == "capture_camera") {
            cap->read(input);
        }
        
        if (input.empty()) {
            cout << "Image empty" << endl;
            return -1;
        }
        
        CircleMarker::findAndEstimate(input, output, _debug, camera, markers, _searchScale);
        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected) {
                cout << markers.at(m).serialize() << endl;
                markers.at(m).detected = false;
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
        #endif
        if (autoproceed && _sourceType == "image_sequence") frame++;
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
    
    ctrl->settings->add("sourceType", &paramSourceType, true);
    ctrl->settings->add("sourceName", &paramSourceName, true);
    ctrl->settings->add("searchScale", &paramSearchScale, true);
    ctrl->settings->add("markerSize", &paramMarkerSize, true);
    ctrl->settings->add("calibrationFile", &paramCalibrationFile, true);
    
    ctrl->settings->add("frameStart", &paramFrameStart, true);
    ctrl->settings->add("frameStop", &paramFrameStop, true);
    ctrl->settings->add("prefix", &paramPrefix, true);
    ctrl->settings->add("suffix", &paramSuffix, true);
    ctrl->settings->add("debug", &paramDebug, true);
    
    ctrl->settings->add("flip", &paramFlip, true);
    ctrl->settings->add("scale", &paramScale, true);
    
    if (!ctrl->settings->load(settingsFile)) {
        cout << "Couldn't find calibration file, writing defaults to " << settingsFile << endl;
        ctrl->settings->save(settingsFile);
    }
    
    bool result = run();
    ctrl->settings->save(settingsFile);
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
 
string paramFlip(int action, string val) {
    if (action == PARAM_SET) {
        _flip = atoi(val.c_str()) == 1;
    } else {
        return _flip ? "1" : "0";
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

string paramCalibrationFile(int action, string val) {
    if (action == PARAM_SET) {
        _calibrationFile = val;
    } else {
        return _calibrationFile;
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
    //imshow("REDUCED", dst);
}
