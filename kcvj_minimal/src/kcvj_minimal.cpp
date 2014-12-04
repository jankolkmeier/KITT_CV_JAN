#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"

using namespace cv;
using namespace std;

Mat input, output;

int main(int argc, char* argv[]) {
	int camera_name = 0;
	int gui = 1;
	string calibration = "camera.yml";

    if (argc != 4) {
       cout << "Options: " << argv[0] << " [src] [calibration] [gui]" << endl;
    } else {
       camera_name = atoi(argv[1]);
       calibration = argv[2];
       gui = atoi(argv[3]);
    } 
    
    cout << "Using camera " << camera_name << " with calibration file " << calibration << " and gui set to " << gui << endl;
    
    VideoCapture cap(camera_name);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    }
    
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, 25.0));
    Camera camera(calibration);
    
    while (true) {
        cap >> input;
        CircleMarker::findAndEstimate(input, output, gui > 0, camera, markers, 0.3);
        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected) {
                cout << markers.at(m).serialize() << endl;
                markers.at(m).detected = false;
            }
        }
        
        if (gui > 0) imshow("OUTPUT", output);
        if (waitKey(30) == 'q') break;
    }
    
    return 0;
}
