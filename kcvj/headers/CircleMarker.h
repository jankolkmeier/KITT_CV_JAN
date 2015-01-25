#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>

#include "Camera.h"
#include "Profiling.h"

using namespace cv;
using namespace std;

void WriteCircleMarkerProfileHeader(ofstream & o);
void WriteCircleMarkerProfileLine(ofstream & o);

void eulerAngles(Mat& R, Mat& r);
double rad2deg(double d);

/**
 * Suspected Circle Marker class used internally by the marker detection algorithm.
 */
class SuspectedCircleMarker {
public:
    int cx;
    int cy;
    int size;
    bool detected_as_horizontal;
    int score_horizontal;
    int score_vertical;
    
    vector<int> cols;
    vector<int> rows;
    
    SuspectedCircleMarker(int x, int y, int s, bool h);
    bool isClose(SuspectedCircleMarker & scm);
    bool tooClose(SuspectedCircleMarker & scm);
    void merge(SuspectedCircleMarker & scm);
    Point3i export_point();
};

/**
 * Circle Marker class providing methods to detect markers in OpenCV Mat's.
 */
class CircleMarker {
public:
    /**
     * Main constructor.
     * @param markerId denoting the ID of the marker (currently, only 0 is supported)
     * @param size denoting the size in world units of the marker.
     */
    CircleMarker(int markerId, double size);
    /**
     * Main constructor with manual world translation and rotation.
     * @see CircleMarker(int markerId, double size)
     * @param t Marker world pose translation vector
     * @param R Marker world pose rotation vector
     */
    CircleMarker(int markerId, double size, Mat &t, Mat &R);
    
    /**
     * Marker ID
     */
    int markerId;
    /**
     * Marker size
     */
    double size;
    
    // Marker pose
    /**
     * 3x1 Rod. Rotation Vector
     * Used for marker pose.
     */
    Mat r_marker;
    
    /**
     * 3x3 Rotation Matrix
     * Used for marker pose.
     */
    Mat R_marker;
    
    /**
     * 3x1 Position Vector
     * Used for marker pose.
     */
    Mat t_marker;
    
    /**
     * 3x3 Rotation Matrix
     * Used for marker pose.
     */
    Mat R_marker_world;
    
    /**
     * 3x1 Position Vector
     * Used for marker pose.
     */
    Mat t_marker_world;
    
    // Camera pose. These vars
    /**
     * 3x3 Rotation Matrix.
     * Used for camera pose and would have to be moved for a multilple cameras/markers implementation to reflect the many-to-many relationship better.
     */
    Mat R_cam;
    
    /**
     * 3x1 Position Vector.
     * Used for camera pose and would have to be moved for a multilple cameras/markers implementation to reflect the many-to-many relationship better.
     */
    Mat t_cam;
    
    /**
     * 3x3 Rotation Matrix.
     * Used for camera pose and would have to be moved for a multilple cameras/markers implementation to reflect the many-to-many relationship better.
     */
    Mat R_camera_world;
    
    /**
     * 3x1 Position Vector.
     * Used for camera pose and would have to be moved for a multilple cameras/markers implementation to reflect the many-to-many relationship better.
     */
    Mat t_camera_world;
    
    /**
     * Manually the world position of the marker.
     * @param t Marker world pose translation vector
     * @param R Marker world pose rotation vector
     */
    void setWorldPose(Mat &t, Mat &R);
    
    /**
     * Serialize the current pose of the marker.
     * @return a string with serialized pose
     */
    string serialize();
    
    /**
     * Find and estimate a marker in the input Mat.
     * @param img The input image
     * @param output Output image for visual debugging
     * @param gui use GUI/debug
     * @param camera Camera properties
     * @param previous vector of previously (un-)detected markers.
     * @param scaleFactor scale factor of the binary search image
     * @param tr threshold value used to create binary search image
     */
    static void findAndEstimate(Mat &img, Mat &output, bool gui, Camera &camera, vector<CircleMarker>& previous, double scaleFactor, int tr);
    
    /**
     * Specification of the corner points.
     */
    vector<Point3d> model;
    
    /**
     * A flag set by findAndEstimate() to indicate whether the marker was detected.
     */
    bool detected;
    
    /**
     * Estimate the pose of the parker based on the scene points.
     * @param scene Pixel coordinates of the corner, in same order as in model
     * @param camera Camera properties
     */
    void estimateMarkerPose(vector<Point2d> & scene, Camera & camera);
    /**
     * Used to debug-draw a detected marker.
     */
    static void drawMaker(CircleMarker & marker, Camera & camera, Mat & output, float calibScaleX, float calibScaleY);
    
    /**
     * Search the circle marker in an image.
     * @param img Binary input image
     * @param circles vector of SuspectedCircleMarker that will be populated by the function
     */
    static void            searchNestedCircles(Mat & img, vector<SuspectedCircleMarker> & circles);
    /**
     * Given a region of interest, find the four outer corners of the CircleMarker.
     * This implementation uses the findContours OpenCV function.
     * @param roi region of interest that completly contains
     * @param offset the offset of the ROI in the original image
     * @param approx the output vector of approximated points
     */
    static void            approximateCornersSlow(Mat & roi, Point offset, vector<Point2i> & approx);
    
    /**
     * Given a region of interest, find the four outer corners of the CircleMarker.
     * This is a placeholder for an yet unimplement, faster/more reliable version.
     * @param roi region of interest that completly contains
     * @param offset the offset of the ROI in the original image
     * @param approx the output vector of approximated points
     */
    static bool            approximateCornersFast(Mat & roi, Point offset, vector<Point2i> & approx);
    
    /**
     * Refine previously approximated corners using a higher-resolution, grayscale image.
     * @param gray A grayscale image
     * @param scaleFactor The scale-factor by which the points in approx are smaller than the grayscale image.
     * @param approx Approximated corner points
     * @return A vector of refined points in the domain of the grayscale image
     */
    static vector<Point2f> refineCorners(Mat & gray, float scaleFactor, vector<Point2i> & approx);
    
    /**
     * Bring the corners in the same order as in model.
     * @param img binary input image
     * @param scale scale factor by which the binary input image was scaled
     * @param corners detected, unsorted corners
     * @param scene sorted output corners
     */
    static bool            sortCorners(Mat & img, double scale, vector<Point2f>& corners, vector<Point2d>& scene);
    
    /**
     * Detect the ID of a given marker whose corner points are already detected.
     * This method is a stub, and allways returns 0.
     * @param img the binary image
     * @param scale scale of the binary image relative to the original input
     * @param scene refined corner points in the scale of the original input
     */
    static int             detectMarkerId(Mat & img, double scale, vector<Point2d>& scene);
private:
    static void            _searchNestedCircles(Mat & row, int offset_x, int offset_y, vector<SuspectedCircleMarker> & bars, bool horizontal);
    void setup(int markerId, double size, Mat &_world_t, Mat &_world_R);
};
