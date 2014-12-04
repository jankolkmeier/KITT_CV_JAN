#include <opencv2/opencv.hpp>
#include <string>

#include "Config.h"
#include "Camera.h"

using namespace cv;
using namespace std;

class CircleMarker {
public:
    CircleMarker(int markerId, double size);
    int markerId;
    double size;
    Mat R; // 3x3 Rotation
    Mat t; // 3x1 Position
    Mat T; // 4x4 Homog. Translation
    string serialize();
    static void findAndEstimate(Mat &img, Camera &camera, vector<CircleMarker>& previous, double scaleFactor);
    vector<Point3d> model;
    bool detected;
private:
    void setMarkerTransform(cv::Mat rvec, cv::Mat tvec);
    static void  searchNestedCircles(Mat & img, vector<Point3i> & circles);
    static bool _searchNestedCircles(Mat & row, int r, vector<Point3i> & bars, bool horizontal);
    static bool  sortCorners(Mat & img, double scale, vector<Point2f>& refined, vector<Point2d>& scene);
    static int   detectMarkerId(Mat & img, double scale, vector<Point2d>& scene);
};

struct SearchState {
    int whites;
    int blacks;
    int state;
    int changes;
    int start_idx;
    int end_idx;
    
    SearchState() {
        whites = 0;
        blacks = 0;
        state = 0;
        changes = 0;
        start_idx = 0;
        end_idx = 0;
    }
};


