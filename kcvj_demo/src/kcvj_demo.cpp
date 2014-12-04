#define KCVJ_GUI 1
#define KCVJ_REMOTE 1

#include "kcvj_demo.h"

int run() {
    
#if (KCVJ_GUI == 1)
#endif
    
    if (!ctrl->imageSet) ctrl->setDebugImage(reduced);
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
    ctrl->settings->add("calibrationFile", &paramCalibrationFile, true);
    
    ctrl->settings->add("frameStart", &paramFrameStart, true);
    ctrl->settings->add("frameStop", &paramFrameStop, true);
    ctrl->settings->add("prefix", &paramPrefix, true);
    ctrl->settings->add("suffix", &paramSuffix, true);
    
    ctrl->settings->add("flip", &paramFlip, false);
    ctrl->settings->add("scale", &paramScale, true);
    
    ctrl->settings->load(settingsFile);
    
    return run();
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
        _frameStart = atoi(val.c_str());
    } else {
        ostringstream buf;
        buf << _frameStart;
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
        _searchScale = min(max(0.05, atof(val.c_str())), 1.0);
    } else {
        ostringstream buf;
        buf << _searchScale;
        return buf.str();
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
        _scale = min(max(0.05, atof(val.c_str())), 1.0);
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
}

/*

int live(int cam, string calibration) {
    VideoCapture cap(cam);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    }
    
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, 25.0));
    Camera camera(calibration);
    
    Mat input;
    while (true) {
        cap >> input;
        // todo: only if new frame?
        CircleMarker::findAndEstimate(input, camera, markers, 0.35);
        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected)
                cout << markers.at(m).serialize() << endl;
        }
        //
        imshow("INPUT", input);
        if (waitKey(30) == 'q') break;
    }
    
    return 0;
}

int files(string prefix, int first, int last, string calibration) {
    int frame = first;
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, 25.0));
    Camera camera(calibration);
    
    bool autoproceed = true;
    Mat input;
    
    while (true) {
        if (frame>last) frame = first;
        
        stringstream ss;
        ss << prefix << frame << ".png";
        input = imread(ss.str(), CV_LOAD_IMAGE_COLOR);
        
        CircleMarker::findAndEstimate(input, camera, markers, 0.35);
        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected) {
                cout << markers.at(m).serialize() << endl;
                markers.at(m).detected = false;
            }
        }
        
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
        if (autoproceed) frame++;
    }
    
    return 0;
}
*/
