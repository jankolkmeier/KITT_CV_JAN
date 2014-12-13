#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"

using namespace cv;
using namespace std;

Mat input, output;

int main(int argc, char* argv[]) {
    int camera_name = 0;
    string calibration = "camera.yml";
	int gui = 0;
    int thresh = 75;
    double markerSize = 25.0;
    
    VideoCapture cap(camera_name);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    }
    
    Camera camera(calibration);
    CircleMarker marker0(0, markerSize);
    vector<CircleMarker> markers;
    markers.push_back(marker0);
                            
    cap.set(CV_CAP_PROP_FRAME_WIDTH, camera.width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
    //cap.set(CV_CAP_PROP_FPS, 10);
    
    cout << "Using camera " << camera_name << " with calibration file " << calibration;
    cout << " and gui " << (gui>0 ? "enabled" : "disabled") << endl;
    
    while (true) {
        cap >> input;
        CircleMarker::findAndEstimate(input, output, gui > 0, camera, markers, 0.3, thresh);
        if (marker0.detected) {
            cout << marker0.serialize() << endl;
            marker0.detected = false;
        } else {
            cout <<  " - " << endl;
        }
        
        if (gui > 0) {
            imshow("OUTPUT", output);
            if (waitKey(30) == 'q') break;
        }
    }
    
    return 0;
}
