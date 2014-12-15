#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"

using namespace cv;
using namespace std;

Mat input, src_gray;

int main(int argc, char* argv[]) {
    int camera_name = 0;
    string calibration = "logitech800x600.yml";
    
    Camera camera(calibration);
    VideoCapture cap(camera_name);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    } else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH, camera.width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
    }
    cout << "Using camera " << camera_name << " with calibration file " << calibration;

    unsigned int frames_total = 0;
    int64 t0_total = GetTimeMs64();
    int64 t0 = GetTimeMs64();;
    
    while (true) {
        cap >> input;
        int t_grab = (int) (GetTimeMs64() - t0);

        t0 = GetTimeMs64();
        
        cvtColor( input, src_gray, CV_BGR2GRAY );
        
        /// Reduce the noise so we avoid false circle detection
        GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
        
        vector<Vec3f> circles;
        
        /// Apply the Hough Transform to find the circles
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
        
        int t_find = (int) (GetTimeMs64() - t0);
        
        /*
        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ ) {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( input, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( input, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }
        imshow("OUTPUT", input);
        if (waitKey(10) == 'q') break;
         */
        
        frames_total++;
        t0 = GetTimeMs64();
        cout << "Avg FPS:" << 1000.0/((unsigned int) ((t0-t0_total)/frames_total)) << ", ";
        cout << "Grab: " << t_grab << "ms, Find: " << t_find << "ms" << endl;
    }
    
    return 0;
}
