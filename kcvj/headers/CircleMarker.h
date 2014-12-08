#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

#include "Camera.h"
#include "Profiling.h"

using namespace cv;
using namespace std;

void WriteCircleMarkerProfileHeader(ofstream & o);
void WriteCircleMarkerProfileLine(ofstream & o);

class SuspectedCircleMarker {
public:
    int cx;
    int cy;
    int size;
    bool detected_as_horizontal;
    int score_horizontal;
    int score_vertical;
    
    SuspectedCircleMarker(int x, int y, int s, bool h);
    bool isClose(SuspectedCircleMarker & scm);
    bool tooClose(SuspectedCircleMarker & scm);
    void merge(SuspectedCircleMarker & scm);
    Point3i export_point();
};

class CircleMarker {
public:
    CircleMarker(int markerId, double size);
    int markerId;
    double size;
    Mat rvec;
    Mat tvec;
    Mat R; // 3x3 Rotation
    Mat t; // 3x1 Position
    Mat T; // 4x4 Homog. Translation
    string serialize();
    static void findAndEstimate(Mat &img, Mat &output, bool debug, Camera &camera, vector<CircleMarker>& previous, double scaleFactor);
    vector<Point3d> model;
    bool detected;
    void reestimateMarker(vector<Point2d> & scene, Camera & camera);
    static void drawMaker(CircleMarker & marker, Camera & camera, Mat & output);
    static void            searchNestedCircles(Mat & img, vector<Point3i> & circles);
    static bool            approximateCornersSlow(Mat & roi, Point offset, vector<Point2i> & approx);
    static bool            approximateCornersFast(Mat & roi, Point offset, vector<Point2i> & approx);
    static vector<Point2f> refineCorners(Mat & gray, float scaleFactor, vector<Point2i> & approx);
    static bool            sortCorners(Mat & img, double scale, vector<Point2f>& refined, vector<Point2d>& scene);
    static int             detectMarkerId(Mat & img, double scale, vector<Point2d>& scene);
    
private:
    static bool            _searchNestedCircles(Mat & row, int offset_x, int offset_y, vector<SuspectedCircleMarker> & bars, bool horizontal);
};
