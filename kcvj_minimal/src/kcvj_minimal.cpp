#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"

using namespace cv;
using namespace std;

Mat input, output;

int main(int argc, char* argv[]) {
    int camera_name = 0;
    string calibration = "logitech800x600.yml";
	int gui = 0;
    int thresh = 75;
    double markerSize = 25.0;
    
    Camera camera(calibration);
    VideoCapture cap(camera_name);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    } else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH, camera.width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
    }
    
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, markerSize));
    CircleMarker * marker0 = &(markers.at(0));

    
    cout << "Using camera " << camera_name << " with calibration file " << calibration;
    cout << " and gui " << (gui>0 ? "enabled" : "disabled") << endl;

    unsigned int frames_total = 0;
    int64 t0_total = GetTimeMs64();
    int64 t0 = GetTimeMs64();;
    
    while (true) {
        cap >> input;
        int t_grab = (int) (GetTimeMs64() - t0);

        t0 = GetTimeMs64();

        CircleMarker::findAndEstimate(input, output, gui>0, camera, markers, 0.3, thresh);
        int t_find = (int) (GetTimeMs64() - t0);

        if (marker0->detected) {
            cout << marker0->serialize() << endl;
            marker0->detected = false;
        } else {
            cout <<  "T: - " << endl << "R: - " << endl;
        }
        
        if (gui > 0) {
            imshow("OUTPUT", output);
            if (waitKey(10) == 'q') break;
        }

        frames_total++;
        t0 = GetTimeMs64();
        cout << "Avg FPS:" << 1000.0/((unsigned int) ((t0-t0_total)/frames_total)) << ", ";
        cout << "Grab: " << t_grab << "ms, Find: " << t_find << "ms" << endl;
    }
    
    return 0;
}
