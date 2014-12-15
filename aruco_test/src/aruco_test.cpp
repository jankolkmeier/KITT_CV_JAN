#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "CircleMarker.h"
#include "aruco.h"
#include <sstream>
#include <ctime>

using namespace cv;
using namespace std;

Mat input;

int main(int argc, char* argv[]) {
    int camera_name = 0;
    string calibration = "logitech800x600.yml";
    Camera camera(calibration);
    Size captureSize(camera.width, camera.height);
    
    vector<Point2f> imagePoints;
    vector<aruco::Marker> markers;
    aruco::MarkerDetector mDetector;
    mDetector.setDesiredSpeed(0);
    mDetector.setMinMaxSize(0.01, 0.75);
    aruco::CameraParameters camParam;
    camParam.setParams(camera.cameraMatrix, camera.distCoeffs, captureSize);
    
    VideoCapture cap(camera_name);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        
        return -1;
    } else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH,  camera.width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
    }
    
    unsigned int frames_total = 0;
    int64 t0_total = GetTimeMs64();
    int64 t0 = GetTimeMs64();;
    
    while (true) {
        cap >> input;
        int t_grab = (int) (GetTimeMs64() - t0);
        
        t0 = GetTimeMs64();

        mDetector.detect(input, markers, camParam);
        int t_find = (int) (GetTimeMs64() - t0);
        
        t0 = GetTimeMs64();
        
        frames_total++;
        cout << "Avg FPS:" << 1000.0/((unsigned int) ((t0-t0_total)/frames_total)) << ", ";
        cout << "Grab: " << t_grab << "ms, Find: " << t_find << "ms" << endl;
    }
    return 0;
}