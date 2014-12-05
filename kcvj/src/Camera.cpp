#include "Camera.h"

Camera::Camera(cv::Mat cm, cv::Mat dc) {
    cameraMatrix = cm;
    distCoeffs = dc;
}

Camera::Camera(string calib_name) {
    FileStorage fs(calib_name, FileStorage::READ);
    if (fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
    } else {
        cout << "Cannot open calibration file" << endl;
    }
    fs.release();
}