#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;
using namespace std;

class Camera {
public:
    Camera(Mat cm, Mat dc);
    Camera(string calib_name);
    Mat cameraMatrix, distCoeffs;
    int width;
    int height;
};