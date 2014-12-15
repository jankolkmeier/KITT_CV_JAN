#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "CircleMarker.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
    int camera_name = 0;
    string calibration = "logitech800x600.yml";
	
    Mat img_object = imread("../../resources/tests/features/features_test2.png", IMREAD_GRAYSCALE);
    if (!img_object.data) {
        cout << "Failed to read test image" << endl;
    }
    Mat input, img_scene;
    
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
    
    
    OrbFeatureDetector detector(100, 2, 2, 31, 0, 2, ORB::HARRIS_SCORE, 31);
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    detector.detect(img_object, keypoints_object);
    OrbDescriptorExtractor extractor(detector);
    BFMatcher matcher(NORM_HAMMING2);
    
    vector< DMatch > matches;
    
    Mat descriptors_object, descriptors_scene;
    extractor.compute(img_object, keypoints_object, descriptors_object);
    
    std::vector<Point2f> obj_corners(4);
    std::vector<Point2f> scene_corners(4);
    
    
    unsigned int frames_total = 0;
    int64 t0_total = GetTimeMs64();
    int64 t0 = GetTimeMs64();;
    
    while (true) {
        cap >> input;
        int t_grab = (int) (GetTimeMs64() - t0);
        cvtColor(input, img_scene, CV_BGR2GRAY);

        t0 = GetTimeMs64();
        detector.detect(img_scene, keypoints_scene);
        extractor.compute(img_scene, keypoints_scene, descriptors_scene);
        int t_extract = (int) (GetTimeMs64() - t0);
        
        t0 = GetTimeMs64();
        if (!descriptors_scene.empty()) {
            matcher.match(descriptors_object, descriptors_scene, matches);
        } else continue;
        
        int t_match = (int) (GetTimeMs64() - t0);
        
        double max_dist = 0;
        double min_dist = 1000000;
        
        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_object.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        
        vector< DMatch > good_matches;
        for (int i = 0; i < descriptors_object.rows; i++) {
            if (matches[i].distance <= max(2*min_dist, 0.02)) {
                good_matches.push_back(matches[i]);
            }
        }
        
        if (good_matches.size() < 4) continue;
        
        Mat img_matches;
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        int m_size =  good_matches.size();
        
        for (size_t i = 0; i < m_size; i++) {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[ good_matches[i].trainIdx ].pt);
            scene.push_back(keypoints_scene[ good_matches[i].queryIdx ].pt);
        }
        
        if (scene.size() > 3) {
            Mat H = findHomography(obj, scene, RANSAC);
            //-- Get the corners from the image_1 ( the object to be "detected" )
            obj_corners[0] = Point(0, 0);
            obj_corners[1] = Point(img_object.cols, 0);
            obj_corners[2] = Point(img_object.cols, img_object.rows);
            obj_corners[3] = Point(0, img_object.rows);
            
            perspectiveTransform(obj_corners, scene_corners, H);
        }
        
        Point2f offset((float) img_object.cols, 0);
        line(img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar(0, 255, 0), 4);
        
        
        imshow("Output", img_matches);
        if (waitKey(10) == 'q') {
            break;
        }
        
        frames_total++;
        t0 = GetTimeMs64();
        cout << "Avg FPS:" << 1000.0/((unsigned int) ((t0-t0_total)/frames_total)) << ", ";
        cout << "Grab: " << t_grab << "ms, Extract: " << t_extract << "ms, Match: " << t_match << endl;
        
    }
    
    return 0;
}
