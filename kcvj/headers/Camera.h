#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;
using namespace std;

class Camera {
public:
    Camera(Mat& cm, Mat& dc);
    Camera(Mat& cm, Mat& dc, Mat& _rig_t, Mat& _rig_R);
    Camera(string calib_name);
    Camera(string calib_name, Mat& _rig_t, Mat& _rig_R);
    Mat cameraMatrix, distCoeffs, rig_R, rig_t;
    int width;
    int height;
private:
    void setup(string calib_name, Mat& _rig_t, Mat& _rig_R);
    void setup(Mat& cm, Mat& dc, Mat& _rig_t, Mat& _rig_R);
};