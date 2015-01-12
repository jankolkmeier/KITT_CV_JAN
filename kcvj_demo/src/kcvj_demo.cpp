#include "kcvj_demo.h"

int run() {
    of.open(_outFile.c_str());
    OutputHeader(of);
    
    VideoCapture * cap;
    
    int frame = _frameStart;
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, _markerSize));
    
    Camera camera(_calibrationFile);
    if (camera.cameraMatrix.empty()) return -1;
    
    cout << "Calibration file: " << _calibrationFile << endl;
    
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

        CircleMarker::findAndEstimate(input_img, output, _gui, camera, markers, _searchScale, _threshold);
        
        o_detected = (markers.at(0).detected ? 1 : 0);
        o_frame = frame;
        o_x = markers.at(0).t_cam.at<double>(0);
        o_y = markers.at(0).t_cam.at<double>(1);
        o_z = markers.at(0).t_cam.at<double>(2);
        
        OutputLine(of);
        
        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected) {
                _translation = markers.at(m).serialize();
				cout << _translation << endl;
				send(_translation);
                markers.at(m).detected = false;
            } else {
                //cout << " - "  << endl;
            }
        }

        if (_gui || _debug) {
			
		#ifndef WIN32
            if (!ctrl->imageSet) {
                reduceImage(output, reduced, _scale);
                ctrl->setDebugImage(reduced);
            } else if (ctrl->image_requested == 1) {
                reduceImage(output, reduced, _scale);
                pthread_mutex_lock(&(ctrl->mutex));
                ctrl->image_requested = 2;
                pthread_mutex_unlock(&(ctrl->mutex));
            }
		#endif
            
            if (_gui) {
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
                    
                    if (k == 'o') {
                        // Set origin
                        Mat t_offset = markers.at(0).t_cam.clone();
                        markers.at(0).setWorldPose(t_offset, markers.at(0).R_marker);
                    }
                    
                    if (k == 'r') {
                        // Reset origin
                        Mat _zero_t = Mat(3, 1, DataType<double>::type);
                        Mat _identity_R = Mat::eye(3, 3, DataType<double>::type);
                        markers.at(0).setWorldPose(_zero_t, _identity_R);
                    }
                    
                    if (k == 'n') {
                        frame++;
                        autoproceed = false;
                        break;
                    }
                } while (!autoproceed);
            }
        }
        if (frame >= _frameStop && _frameStop != 0) return 1;
        if (autoproceed) frame++;
    }
    
    return 0;
}

// Init settings
int main(int argc, char* argv[]) {
    if (argc == 1) {
        cout << "Using default port (" << port << ") and settings file (";
        cout <<  settingsFile << ")" << endl;
    } else if (argc == 2) {
       port = atoi(argv[1]);
    } else if (argc == 3) {
       port = atoi(argv[1]);
       settingsFile = argv[2];
    } else if (argc == 3) {
       port = atoi(argv[1]);
       settingsFile = argv[2];
	   poseTargetPort = atoi(argv[3]);
    } else if (argc == 3) {
       port = atoi(argv[1]);
       settingsFile = argv[2];
       poseTargetPort = atoi(argv[3]);
	   poseTargetIP = argv[4];
    } else {
       cout << "Usage: " << argv[0] << " [port] [settingsFile]" << endl;
       return -1;
    }

	if (initNet() < 0) {
		cout << "Failed to init pose streaming" << endl;
		return -1;
	}
    
    // Init remote control...
    ctrl = new RemoteControl(port, settingsFile);
    
    
    // ...and settigns callbacks
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
    ctrl->settings->add("gui", &paramGUI, true);
    
    // If we cannot load from a settigns file, save defaults to a new one instead
    if (!ctrl->settings->load(settingsFile)) {
        cout << "Couldn't find calibration file, writing defaults to " << settingsFile << endl;
        ctrl->settings->save(settingsFile);
    }
    
    // Run
    bool result = run();
    
    // At the end, save any settings that were changed at runtime
    ctrl->settings->save(settingsFile);
    of.close();

#ifdef WIN32
    closesocket(s);
    WSACleanup();
#else
	close(s);
#endif

    return result;
}

int initNet() {
    printf("\nInitialising Sock...\n");
#ifdef WIN32
    if (WSAStartup(MAKEWORD(2,2),&wsa) != NO_ERROR) {
        printf("WinSock Startup failed. Error Code: %d\n",WSAGetLastError());
        return -1;
    }
#else
    int errno;
#endif

    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
#ifdef WIN32
        printf("socket() failed with error code: %d\n" , WSAGetLastError());
#else
        cout << "socket() failed with error code: " << errno << endl;
#endif
        return -1;
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
	si_other.sin_port = htons(poseTargetPort);
#ifdef WIN32
	si_other.sin_addr.S_un.S_addr = inet_addr(poseTargetIP);
#else
    si_other.sin_addr.s_addr = inet_addr(poseTargetIP);
#endif
    printf("Initialised.\n");
    return 1;
}

void send(string msg) {
#ifndef WIN32
    int errno;
#endif
    msg.copy(message, msg.length());
    if (sendto(s, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR) {
#ifdef WIN32
        printf("sendto() failed with error code : %d" , WSAGetLastError());
#else
        cout << "send() failed with error code: " << errno << endl;
#endif
    }
    memset(buf,'\0', BUFLEN);
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

string paramGUI(int action, string val) {
    if (action == PARAM_SET) {
        _gui = atoi(val.c_str()) == 1;
    } else {
        return _gui ? "1" : "0";
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

// Create a scaled image to send over as dbg        
void reduceImage(Mat &src, Mat &dst, float scale) {
    if (src.empty()) return;
    Mat tmp;
    cvtColor(src, tmp, CV_RGB2GRAY);
    if (_flip)
        flip(tmp, tmp, 1);
    resize(tmp, dst, Size(), scale, scale, INTER_NEAREST);
}

// For logging
void OutputHeader(ofstream & o) {
    o << "t,frame,detected,x,y,z,t_grab,";
    WriteCircleMarkerProfileHeader(o);
    o << endl;
}

// For logging
void OutputLine(ofstream & o) {
    int o_t = (int)(GetTimeMs64() - time_start);
    o << o_t << "," << o_frame << "," << o_detected << "," << o_x << "," << o_y << "," << o_z << "," << t_grab << ",";
    WriteCircleMarkerProfileLine(o);
    o << endl;
}
