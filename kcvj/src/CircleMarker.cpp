#include "CircleMarker.h"

CircleMarker::CircleMarker(int markerId, double size) {
    this->markerId = 0;
    this->size = size;
    this->rvec = Mat(3,1,DataType<double>::type);
    this->tvec = Mat(3,1,DataType<double>::type);
    this->T = Mat(4, 4, DataType<double>::type);
    this->R = Mat(3, 3, DataType<double>::type);
    this->t = Mat(3, 1, DataType<double>::type);
    this->r = Mat(3, 1, DataType<double>::type);
    double *p = T.ptr<double>(3);
    p[0] = p[1] = p[2] = 0; p[3] = 1;
    
    double a = size / 2.0;
    this->model.push_back(Point3d(-a,  a, 0));
    this->model.push_back(Point3d( a,  a, 0));
    this->model.push_back(Point3d( a, -a, 0));
    this->model.push_back(Point3d(-a, -a, 0));
    this->detected = false;
}

void CircleMarker::reestimateMarker(vector<Point2d> & scene, Camera & camera) {
    solvePnP(model, scene, camera.cameraMatrix, camera.distCoeffs, rvec, tvec, false, CV_ITERATIVE);// CV_P3P, CV_EPNP, CV_ITERATIVE
    detected = true;
    Rodrigues(rvec, R);
    R = R.t();  // rotation of inverse
    eulerAngles(R, r);
    t = -R * tvec; // translation out (inverse)
    //Rodrigues(R, r); // rotation out
    T(Range(0,3), Range(0,3)) = R * 1; // copies R into T
    T(Range(0,3), Range(3,4)) = t * 1; // copies tvec into T
}


string CircleMarker::serialize() {
    ostringstream buf;
    buf << "T: " << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2) << "\n";
    buf << "R: " << rad2deg(r.at<double>(0)) << " " << rad2deg(r.at<double>(1)) << " " << rad2deg(r.at<double>(2));
    return buf.str();
}

int t_total = 0;
int t_prepare = 0;
int t_search = 0;
int t_approx = 0;
int t_refine = 0;
int t_estimate = 0;

void WriteCircleMarkerProfileHeader(ofstream & o) {
    o << "t_total,t_prepare,t_search,t_approx,t_refine,t_estimate";
}

void WriteCircleMarkerProfileLine(ofstream & o) {
    o << t_total << "," << t_prepare << "," << t_search << "," << t_approx << "," << t_refine << "," << t_estimate;
}


void CircleMarker::findAndEstimate(Mat &img, Mat &output, bool debug, Camera &camera, vector<CircleMarker> &markers, double scaleFactor, int tr) {
    t_prepare = 0;
    t_search = 0;
    t_approx = 0;
    t_refine = 0;
    t_estimate = 0;
    t_total = 0;
    
    int64 t0_t = GetTimeMs64();
    int64 t0 = GetTimeMs64();
    
    Mat gray, bw;
    cvtColor(img, gray, CV_BGR2GRAY);
    resize(gray, bw, Size(), scaleFactor, scaleFactor);
    threshold(bw, bw, tr, 255, THRESH_BINARY_INV);
    t_prepare += (int)(GetTimeMs64() - t0);
    
    t0 = GetTimeMs64();
    vector<SuspectedCircleMarker> circles;
    searchNestedCircles(bw, circles);
    t_search += (int)(GetTimeMs64() - t0);

    output = img;
    
    if (debug) {
        Mat bwc;
        cvtColor(bw, bwc, CV_GRAY2RGB);
        bwc.copyTo(output(Rect(0, 0, bwc.cols, bwc.rows)));
    }

    for (int i = 0; i < circles.size(); i++) {
        Point center(circles[i].cx, circles[i].cy);
        // Width of one marker is 13 single rings (4 outer rings, center rings, 2x 2 ring-widths for clearance).
        // the detected width in circles[i].z equals to the circles. to get a pixel estimate for the clearance
        // we multiply it by
        double marker_side = (circles[i].size/9.0)*13.0;
        // Now we can calculate the hypothenuse of the square to find an area that encloses the
        // complete marker for any rotation. We also add some 15% to prevent measurement errors.
        int range = sqrt(2.0*(marker_side*marker_side))*1.15; // hypothenuse of circle + some extra 10%
        
        int x1 = max(0, circles[i].cx-range);
        int y1 = max(0, circles[i].cy-range);
        int x2 = min(bw.size().width, circles[i].cx+range);
        int y2 = min(bw.size().height, circles[i].cy+range);
        Mat roi = bw(Range(y1, y2), Range(x1, x2));
        
        if (debug) {
            for (int r=0; r<circles[i].rows.size(); r++) {
                line(img, Point(0, circles[i].rows[r]), Point(bw.cols, circles[i].rows[r]), Scalar(30,70,250), 1, 1);
            }
            for (int c=0; c<circles[i].cols.size(); c++) {
                line(img, Point(circles[i].cols[c], 0), Point(circles[i].cols[c], bw.rows), Scalar(250,70,30), 1, 1);
            }
            rectangle(img, Point(x1,y1),  Point(x2,y2),  Scalar(180,30,220), 1, 1);
            circle(img, center, circles[i].size, CV_RGB(0,255,255), 1);
        }
        
        t0 = GetTimeMs64();
        // Try approximate corners in outline
        vector<Point2i> approx;
        approximateCornersSlow(roi, Point(x1, y1), approx);
        t_approx += (int)(GetTimeMs64() - t0);
        
        if (approx.size() != 4) break;
        
        vector<Point2f> refined = refineCorners(gray, scaleFactor, approx);
        vector<Point2d> scene; // Marker corners in scene, order corresponding with model.
        
        t0 = GetTimeMs64();
        // Can we associate the corners with the model?
        bool foundRefined = sortCorners(bw, scaleFactor, refined, scene);
        t_refine += (int)(GetTimeMs64() - t0);
        if (foundRefined) {
            // Can we detect the marker id? (this is still a stub/allways returns 0)
            int markerId = detectMarkerId(bw, scaleFactor, scene);
            if (markerId > -1) {
                for (int midx = 0; midx < markers.size(); midx++) {
                    // Do we have that marker in our
                    if (markers.at(midx).markerId == markerId) {
                        CircleMarker * marker = &markers.at(midx);
                        t0 = GetTimeMs64();
                        
                        Size res = img.size();
                        float calibScaleX = camera.width/res.width;
                        float calibScaleY = camera.height/res.height;
                        
                        //for (int p = 0; p < scene.size(); p++) {
                        //    scene[p].x = scene[p].x * calibScaleX;
                        //    scene[p].x = scene[p].y * calibScaleY;
                        //}
                        
                        marker->reestimateMarker(scene, camera);
                        t_estimate += (int)(GetTimeMs64() - t0);
                        if (debug) drawMaker(*marker, camera, output, calibScaleX, calibScaleY);
                        break;
                    }
                }
            }
        }
    }
    t_total += (int)(GetTimeMs64() - t0_t);
}

// Slow implementation using built-in opencv functions
void CircleMarker::approximateCornersSlow(Mat & roi, Point offset, vector<Point2i> & approx) {
    // Inside the roi, we walk the extreme outer contour
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(roi, contours, hierarchy,
                 CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, offset);
    
    double max_area = 0;
    
    // If there any shapes inside (>0) and if it is not too "noisy" (<12)
    if (contours.size() > 0 && contours.size() < 12) {
        for (int ci=0; ci < contours.size(); ci++) {
            vector<Point2i> _approx;
            approxPolyDP(contours[ci], _approx, 10, true); // small epsilon = detailed approx.size()
            // TODO: calculate area, return the contour with max(A) instead
            if (_approx.size() == 4) {
                double A = contourArea(_approx);
                if (A > max_area) {
                    max_area = A;
                    approx = _approx;
                }
            }
        }
    }
}

// Not implemented yet, using findContours +
bool CircleMarker::approximateCornersFast(Mat & roi, Point offset, vector<Point2i> & approx) {
    Point2i ptl(roi.cols/2, roi.rows/2);
    Point2i ptr(roi.cols/2, roi.rows/2);
    Point2i pbr(roi.cols/2, roi.rows/2);
    Point2i pbl(roi.cols/2, roi.rows/2);
    
    approx.clear();
    approx.push_back(ptl);
    approx.push_back(ptr);
    approx.push_back(pbr);
    approx.push_back(pbl);
    
    return false;
}

// Approx = 4 corners, to be refined in gray img, which is bigger by scaleFactor
vector<Point2f> CircleMarker::refineCorners(Mat & gray, float scaleFactor, vector<Point2i> & approx) {
    vector<Point2f> refined;
    Mat(approx).copyTo(refined);
    
    for (int c = 0; c < refined.size(); c++) {
        refined[c].x = refined[c].x/scaleFactor;
        refined[c].y = refined[c].y/scaleFactor;
    }
    
    TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.001 );
    cornerSubPix(gray, refined, Size(6,6), Size(-1,-1), criteria);
    
    return refined;
}

void CircleMarker::drawMaker(CircleMarker & marker, Camera & camera, Mat & output, float calibScaleX, float calibScaleY) {
    // Draw reprojected marker & COS
    vector<Point3f> cosx; vector<Point2f> cosx_img;
    vector<Point3f> cosy; vector<Point2f> cosy_img;
    vector<Point3f> cosz; vector<Point2f> cosz_img;
    vector<Scalar> cornerColors;
    cornerColors.push_back(Scalar(255,0,0));
    cornerColors.push_back(Scalar(0,255,0));
    cornerColors.push_back(Scalar(0,0,255));
    cornerColors.push_back(Scalar(255,0,255));
    cosx.push_back(Point3f(0,0,0));
    cosx.push_back(Point3f(marker.size/2,0,0));
    cosy.push_back(Point3f(0,0,0));
    cosy.push_back(Point3f(0,marker.size/2,0));
    cosz.push_back(Point3f(0,0,0));
    cosz.push_back(Point3f(0,0,marker.size/2));
    std::vector<Point2d> scene_corners(4);
    projectPoints(marker.model, marker.rvec, marker.tvec, camera.cameraMatrix, camera.distCoeffs, scene_corners);
    projectPoints(cosx, marker.rvec, marker.tvec, camera.cameraMatrix, camera.distCoeffs, cosx_img);
    projectPoints(cosy, marker.rvec, marker.tvec, camera.cameraMatrix, camera.distCoeffs, cosy_img);
    projectPoints(cosz, marker.rvec, marker.tvec, camera.cameraMatrix, camera.distCoeffs, cosz_img);
    
    //for (int p = 0; p < scene_corners.size(); p++) {
    //    scene_corners[p].x = scene_corners[p].x / calibScaleX;
    //    scene_corners[p].x = scene_corners[p].y / calibScaleY;
    //}
    
    
    for (int c = 0; c<4; c++) {
        // Approximated corners
        //circle(output, approx[c], 5, cornerColors[c]);
        circle(output, scene_corners[c], 5, cornerColors[c]);
        
        // Reproject corners with found translation; connect with lines
        line(output, scene_corners[c], scene_corners[(c+1)%4], Scalar(255, 0, 255), 1);
    }
    
    // Marker COS
    line(output, cosx_img[0], cosx_img[1], Scalar(255,0,0), 2);
    line(output, cosy_img[0], cosy_img[1], Scalar(0,255,0), 2);
    line(output, cosz_img[0], cosz_img[1], Scalar(0,0,255), 2);
}

bool CircleMarker::sortCorners(Mat & img, double scale, vector<Point2f>& corners, vector<Point2d>& scene) {
    // We want to know the right order of corners to match our marker model.
    // Depending on which corner the white spot is, we want to circular shift the array of indexes below:
    vector<int> cornerOrder;
    cornerOrder.push_back(3);
    cornerOrder.push_back(2);
    cornerOrder.push_back(1);
    cornerOrder.push_back(0);
    
    // The white spot - which markers corner 0 - is located on the diagonal between the two corners.
    // To figure out which corner is the one with the white spot, we interpolate between the two sets of corners.
    // In the model, the center of the white corner spot is at 22.98/212.13 of the diameter
    double rr = 22.98/212.13;
    double ir = 1-rr;
    // We may safely assume that approxDP returns a sorted list of the 4 corners
    // Hence the opposite pairs are:  0 and 2; 1 and 3.
    // We look at the indexes located under the two interpolated spots on the two diagonals:
    if ((uchar)img.at<uchar>((int) (corners[0].y*ir + corners[2].y*rr)*scale, (int) (corners[0].x*ir + corners[2].x*rr)*scale) == 0) {
        rotate(cornerOrder.begin(),cornerOrder.begin()+3,cornerOrder.end());
    } else if ((uchar)img.at<uchar>((int) (corners[1].y*ir + corners[3].y*rr)*scale, (int) (corners[1].x*ir + corners[3].x*rr)*scale) == 0) {
        rotate(cornerOrder.begin(),cornerOrder.begin()+2,cornerOrder.end());
    } else if ((uchar)img.at<uchar>((int) (corners[0].y*rr + corners[2].y*ir)*scale, (int) (corners[0].x*rr + corners[2].x*ir)*scale) == 0) {
        rotate(cornerOrder.begin(),cornerOrder.begin()+1,cornerOrder.end());
    } else if ((uchar)img.at<uchar>((int) (corners[1].y*rr + corners[3].y*ir)*scale, (int) (corners[1].x*rr + corners[3].x*ir)*scale) == 0) {
        rotate(cornerOrder.begin(),cornerOrder.begin()+0,cornerOrder.end());
    } else {
        // If we fail to find a white spot, we cannot procede.
        // This may be because the white spot is occluded or because we detected a non-marker.
        return false;
    }
    
    for (int c = 0; c < 4; c++) {
        scene.push_back(Point2d(corners[cornerOrder.at(c)].x, corners[cornerOrder.at(c)].y));
    }
    return true;
}

SuspectedCircleMarker::SuspectedCircleMarker(int x, int y, int s, bool h) {
    cx = x;
    cy = y;
    size = s;
    detected_as_horizontal = h;
    score_horizontal = 0;
    score_vertical = 0;
    if (detected_as_horizontal) {
        cols.push_back(x);
    } else {
        rows.push_back(y);
    }
}

bool SuspectedCircleMarker::isClose(SuspectedCircleMarker & scm) {
    int tsize = (size + scm.size);
    //float ratio = (float) max(size, scm.size)/(float)tsize;
    return abs(cx - scm.cx) < tsize/5 && abs(cy - scm.cy) < tsize/5;
}

bool SuspectedCircleMarker::tooClose(SuspectedCircleMarker & scm) {
    int msize = (size + scm.size);
    return abs(cx - scm.cx) < msize && abs(cy - scm.cy) < msize;
}

void SuspectedCircleMarker::merge(SuspectedCircleMarker & scm) {
    size = max(size, scm.size);
    if (scm.detected_as_horizontal) {
        cols.push_back(scm.cx);
        score_horizontal++;
    } else {
        rows.push_back(scm.cy);
        score_vertical++;
    }
    
    int samples =(score_horizontal+score_vertical);
    cx = ((cx * samples) + scm.cx ) / (samples + 1);
    cy = ((cy * samples) + scm.cy ) / (samples + 1);
    
    //cx = (cx + scm.cx)/2;
    //cy = (cy + scm.cy)/2;
}

Point3i SuspectedCircleMarker::export_point() {
    return Point3i((int) cx, (int) cy, (int) size);
}

int CircleMarker::detectMarkerId(Mat & img, double scale, vector<Point2d>& scene) {
    return 0;
}

void CircleMarker::searchNestedCircles(Mat & img, vector<SuspectedCircleMarker> & circles) {
    
    Mat dbg_search;
    cvtColor(img, dbg_search, CV_GRAY2BGR);

    Size size = img.size();
    vector<SuspectedCircleMarker> sus_markers;
    
    // Find possible circle markers by looking through rows (horizontal)
    for (int r = 0; r < size.height; r++) {
        Mat rMat = img.row(r);
        _searchNestedCircles(rMat, 0, r, sus_markers, true);
    }
    
    // Remove those that don't have at least two good horizontal samples
    for (vector<SuspectedCircleMarker>::iterator bar=sus_markers.begin(); bar!=sus_markers.end();){
        if(bar->score_horizontal < 2) bar = sus_markers.erase(bar);
        else ++bar;
    }
    
    // Search through columns (vertical) only in vincinity of the leftover suspected markers
    for (int b = 0; b < sus_markers.size(); b++) {
        SuspectedCircleMarker * bar = &sus_markers[b];
        circle(dbg_search, Point2i((int)bar->cx, (int)bar->cy), (int)bar->size, Scalar(0,0,255));
        int range_y = (int)bar->size*2; // center must be ins
        int range_x = (int)bar->size/4; // center must be ins
        int x_offset = max(0, (int)bar->cx-range_x);
        int y_offset = max(0, (int)bar->cy-range_y);
        Mat roi_sus = img(Range(y_offset, min(size.height, (int)bar->cy+range_y)), Range(x_offset, min(size.width, (int)bar->cx+range_x)));
        for (int c = 0; c < roi_sus.cols; c++) {
            Mat rMat = roi_sus.col(c);
            _searchNestedCircles(rMat, c+x_offset, y_offset, sus_markers, false);
        }
    }
    
    // Return only those with at least two vertical and two horizontal samples
    for (int b = 0; b < sus_markers.size(); b++) {
        if (sus_markers[b].score_horizontal > 1 && sus_markers[b].score_vertical > 1) {
            circles.push_back(sus_markers[b]);
        }
    }
}

// 1 [Read blacks]
//    Flop: 2 / possible_start
// 2 [Read whites]
//    Flop_U: 3  (U: ratio blacks/whites is higher *many blacks*)
// 3 [Read black Ring]
//    Flop_S: 4
//    Flop_N: 2
//    NoFlop: 1
// 4 [Read white Ring]
//    Flop_S_X: 1 [Done] / definite_end
//    Flop_S: 3
//    Flop_N: 1
//    NoFlop: 2
//
void CircleMarker::_searchNestedCircles(Mat & row, int offset_x, int offset_y, vector<SuspectedCircleMarker> & bars, bool horizontal) {
    float max_ratio = 0.72f;
    uchar marker_flips = 8;

    uchar flips = 0;
    uchar state = 1;
    ushort possible_start = 0;
    ushort definite_end = 0;
    ushort curr = 0;
    ushort prev = 0;

    bool black = (uchar)row.at<uchar>(0) == 255; // ==255 instead of ==0 because we have inverted img
    bool white = !black;
    
    int end = horizontal ? row.size().width : row.size().height;
    for (int i = 0; i < end; i++) {
        bool next_white = (uchar)row.at<uchar>(i) == 0;

        if (next_white == white) curr++;
        
        white = next_white;
        black = !white;

        float ratio = max(curr, prev)/(float)(curr+prev);

        if (state == 1) { // Read "leading" blacks
            if (white) {
                state = 2;
                prev = curr;
                curr = 1;
                possible_start = i;
            }
        } else if (state == 2) { // Read white ring
            if (black) {
                if (curr < prev) {
                    state = 3;
                    flips = 1;
                } else {
                    state = 1;
                }
                
                prev = curr;
                curr = 1;
            }
        } else if (state == 3) { // Read black ring
            if (white) {
                prev = curr;
                curr = 1;
                if (ratio < max_ratio) {
                    state = 4;
                    flips++;
                } else {
                    state = 2;
                }
            } else if (curr > prev && ratio >= max_ratio) {
                state = 1;
            }
        } else if (state == 4) { // Read white ring
            if (black) {
                definite_end = i-1;
                prev = curr;
                curr = 1;
                if (ratio < max_ratio) {
                    if (flips == marker_flips) {
                        state = 5;
                    } else {
                        state = 3;
                        flips++;
                    }
                } else {
                    state = 1;
                }
            } else if (curr > prev && ratio >= max_ratio) {
                state = 2;
            }
        } else if (state == 5) {
            if (white) {
                if (curr < prev) {
                    state = 1;
                } else {
                    int size = (definite_end - possible_start)/2;
                    int center = possible_start+size;
                    SuspectedCircleMarker current(
                                                  horizontal ? center+offset_x : offset_x,
                                                  horizontal ? offset_y : center+offset_y,
                                                  size, horizontal);
                    
                    bool found = false;
                    int filter = -1;
                    for (int b = 0; b < bars.size(); b++) {
                        SuspectedCircleMarker * other = &bars[b];
                        if (other->isClose(current)) {
                            other->merge(current);
                            found = true;
                            break;
                        } else if (other->tooClose(current)) {
                            filter = b;
                            break;
                        }
                    }
                    
                    // We only add new detected circles in horizontal search
                    if (horizontal) {
                        if (!found) { // ...if none was found closeby
                            bars.push_back(current);
                        }
                        
                        // ...and if it was close, but not close enough, to an actual one circle...
                        if (filter > -1) { // It is probably noise, and we remove it
                            bars.erase(bars.begin()+filter);
                        }
                    }
                }
            } else if (curr > prev && ratio >= max_ratio) {
                state = 1;
            }
        }
    }
}

const double PI = 3.14159265358979323846264;

bool closeEnough(const float& a, const double& b, const double& epsilon = numeric_limits<double>::epsilon()) {
    return (epsilon > abs(a - b));
}

double rad2deg(double d) {
    return d * (180.0 / PI);
}

void eulerAngles(Mat& R, Mat& res) {
    //check for gimbal lock
    if (closeEnough(R.at<double>(2,0), -1.0)) {
        res.at<double>(0) = 0; //gimbal lock, value of x doesn't matter
        res.at<double>(1) = PI / 2;
        res.at<double>(2) = atan2(R.at<double>(0,1), R.at<double>(0,2));
    } else if (closeEnough(R.at<double>(2,0), 1.0)) {
        res.at<double>(0) = 0;
        res.at<double>(1) = -PI / 2;
        res.at<double>(2) = atan2(-R.at<double>(0,1), -R.at<double>(0,2));
    } else {
        float x1 =  -asin(R.at<double>(2,0));
        res.at<double>(0) = x1;
        res.at<double>(1) = atan2(R.at<double>(2,1) / cos(x1), R.at<double>(2,2) / cos(x1));
        res.at<double>(2) = atan2(R.at<double>(1,0) / cos(x1), R.at<double>(0,0) / cos(x1));
    }
}
