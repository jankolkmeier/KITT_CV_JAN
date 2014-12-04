// OpenCVTest3.cpp : Defines the entry point for the console application.
//

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "Config.h"
#include "CircleMarker.h"

using namespace cv;
using namespace std;



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

int main(int argc, char* argv[]) {
    //return live(0, "../../resources/calibration/macbook.yml");
    return files("../../resources/optitrack/test_", 0, 578, "../../resources/calibration/macbook.yml");
}