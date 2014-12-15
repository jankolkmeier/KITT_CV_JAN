#include "Camera.h"

Camera::Camera(cv::Mat & cm, cv::Mat & dc) {
    Mat _t = Mat(3, 1, DataType<double>::type);
    Mat _r = Mat::eye(3, 3, DataType<double>::type);
    setup(cm, dc, _t, _r);
}

Camera::Camera(cv::Mat & cm, cv::Mat & dc, Mat& _rig_t, Mat& _rig_R) {
    setup(cm, dc, _rig_t, rig_R);

}

Camera::Camera(string calib_name, Mat& _rig_t, Mat& _rig_R) {
    setup(calib_name, _rig_t, rig_R);
}

Camera::Camera(string calib_name) {
    Mat _t = Mat(3, 1, DataType<double>::type);
    Mat _r = Mat::eye(3, 3, DataType<double>::type);
    setup(calib_name, _t, _r);
}


void Camera::setup(string calib_name, Mat& _rig_t, Mat& _rig_R) {
    FileStorage fs(calib_name, FileStorage::READ);
    if (fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs["image_width"] >> width;
        fs["image_height"] >> height;
    } else {
        cout << "Cannot open calibration file" << endl;
    }
    
    fs.release();
}

void Camera::setup(Mat& cm, Mat& dc, Mat& _rig_t, Mat& _rig_R) {
    this->rig_R = _rig_R;
    this->rig_t = _rig_t;
    this->cameraMatrix = cm;
    this->distCoeffs = dc;
}