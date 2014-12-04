#include "CircleMarker.h"

CircleMarker::CircleMarker(int markerId, double size) {
    this->markerId = 0;
    this->size = size;
    this->T = Mat(4, 4, DataType<double>::type);
    this->R = Mat(3, 3, DataType<double>::type);
    //this->rout = Mat(3, 1, DataType<double>::type);
    double *p = T.ptr<double>(3);
    p[0] = p[1] = p[2] = 0; p[3] = 1;
    
    double a = size / 2.0;
    this->model.push_back(Point3d(-a,  a, 0));
    this->model.push_back(Point3d( a,  a, 0));
    this->model.push_back(Point3d( a, -a, 0));
    this->model.push_back(Point3d(-a, -a, 0));
    this->detected = false;
}

void CircleMarker::setMarkerTransform(Mat rvec, Mat tvec) {
    Rodrigues(rvec, R);
    R = R.t();  // rotation of inverse
    t = -R * tvec; // translation of inverse
    T(Range(0,3), Range(0,3)) = R * 1; // copies R into T
    T(Range(0,3), Range(3,4)) = t * 1; // copies tvec into T
    //Rodrigues(R, rout);
}


string CircleMarker::serialize() {
    ostringstream buf;
    buf << markerId << " ";
    /*
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (r+c > 0) buf << " ";
            buf << ((int) (T.at<double>(r, c) * 1000));
        }
    }
     */
    
    buf << " " << t.at<double>(0) << " " << t.at<double>(1) << " " << t.at<double>(2);
    return buf.str();
}


void CircleMarker::findAndEstimate(Mat &img, Camera &camera, vector<CircleMarker> &markers, double scaleFactor) {
    Mat gray, bw;
    
    cvtColor(img, gray, CV_BGR2GRAY);
    resize(gray, bw, Size(), scaleFactor, scaleFactor);
    threshold(bw, bw, 75, 255, THRESH_BINARY_INV); // do _INV, then switch black and white in search?
    // ...would save the inverse for outline detection
    
    vector<Point3i> circles;
    searchNestedCircles(bw, circles);
    
    #if (KCVJ_GUI==1)
    Mat bwc;
    Mat output(img);
    cvtColor(bw, bwc, CV_GRAY2RGB);
    bwc.copyTo(output(Rect(0, 0, bwc.cols, bwc.rows)));
    #endif
    
    for (int i = 0; i < circles.size(); i++) {
        Point center(circles[i].x, circles[i].y);
        int range = circles[i].z*2.25;
        int x1 = max(0, circles[i].x-range);
        int y1 = max(0, circles[i].y-range);
        int x2 = min(bw.size().width, circles[i].x+range);
        int y2 = min(bw.size().height, circles[i].y+range);
        Mat roi = bw(Range(y1, y2), Range(x1, x2));
        //roi = Mat::ones(roi.size(), roi.type()) * 255 - roi;
        
        // Inside the roi, we walk the extreme outer contour
        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(roi, contours, hierarchy,
                     CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(x1, y1));
        
        // If there are shapes inside, and if it is not too noisy...
        if (contours.size() > 0 && contours.size() < 4) {
            vector<Point2i> approx;
            for (int ci=0; ci < contours.size(); ci++) {
                #if (KCVJ_GUI==1)
                // Draw Candidate
                rectangle(img, Point(x1,y1),  Point(x2,y2),  Scalar(180,30,220), 1, 1);
                circle(img, center, circles[i].z, CV_RGB(0,255,255), 1);
                drawContours(img, contours, ci, Scalar(255,255,255));
                #endif
                
                // Try to approximate
                approx.clear();
                approxPolyDP(contours[ci], approx, 8, true);
                
                if (approx.size() == 4) {
                    vector<Point2f> refined;
                    Mat(approx).copyTo(refined);
                    
                    for (int c = 0; c < refined.size(); c++) {
                        refined[c].x = refined[c].x/scaleFactor;
                        refined[c].y = refined[c].y/scaleFactor;
                    }
                    
                    TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.001 );
                    cornerSubPix(gray, refined, Size(6,6), Size(-1,-1), criteria);
                    
                    vector<Point2d> scene;
                    if (!sortCorners(bw, scaleFactor, refined, scene)) continue;
                    CircleMarker * marker = NULL;
                    
                    int markerId = detectMarkerId(bw, scaleFactor, scene);
                    for (int midx = 0; midx < markers.size(); midx++) {
                        if (markers.at(midx).markerId == markerId) marker = &markers.at(midx);
                    }
                    if (marker == NULL) continue; // Marker not used
                    
                    Mat rvec = Mat(3,1,DataType<double>::type);
                    Mat tvec = Mat(3,1,DataType<double>::type);
                    solvePnP(marker->model, scene, camera.cameraMatrix, camera.distCoeffs, rvec, tvec, false, CV_ITERATIVE);// CV_P3P, CV_EPNP, CV_ITERATIVE
                    marker->setMarkerTransform(rvec, tvec);
                    marker->detected = true;
                    
                    
                    
                    
                    
                    
                    
                    #if (KCVJ_GUI==1)
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
                    cosx.push_back(Point3f(marker->size/2,0,0));
                    cosy.push_back(Point3f(0,0,0));
                    cosy.push_back(Point3f(0,marker->size/2,0));
                    cosz.push_back(Point3f(0,0,0));
                    cosz.push_back(Point3f(0,0,marker->size/2));
                    std::vector<Point2d> scene_corners(4);
                    projectPoints(marker->model, rvec, tvec, camera.cameraMatrix, camera.distCoeffs, scene_corners);
                    projectPoints(cosx, rvec, tvec, camera.cameraMatrix, camera.distCoeffs, cosx_img);
                    projectPoints(cosy, rvec, tvec, camera.cameraMatrix, camera.distCoeffs, cosy_img);
                    projectPoints(cosz, rvec, tvec, camera.cameraMatrix, camera.distCoeffs, cosz_img);
                    
                    for (int c = 0; c<4; c++) {
                        // Approximated corners
                        circle(output, approx[c], 5, cornerColors[c]);
                        circle(output, scene_corners[c], 5, cornerColors[c]);
                        
                        // Reproject corners with found translation; connect with lines
                        line(output, scene_corners[c], scene_corners[(c+1)%4], Scalar(255, 0, 255), 1);
                    }
                    
                    // Marker COS
                    line(output, cosx_img[0], cosx_img[1], Scalar(255,0,0), 2);
                    line(output, cosy_img[0], cosy_img[1], Scalar(0,255,0), 2);
                    line(output, cosz_img[0], cosz_img[1], Scalar(0,0,255), 2);
                    #endif
                    
                    
                }
            }
        }
    }
    #if (KCVJ_GUI==1)
    imshow("Output", output);
    #endif
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

int CircleMarker::detectMarkerId(Mat & img, double scale, vector<Point2d>& scene) {
    return 0;
}

void CircleMarker::searchNestedCircles(Mat & img, vector<Point3i> & circles) {
    Size size = img.size();
    vector<Point3i> horiz_bars;
    
    // Look for (grouped) bars in rows
    for (int r = 0; r < size.height; r++) {
        Mat rMat = img.row(r);
        CircleMarker::_searchNestedCircles(rMat, r, horiz_bars, true);
    }
    
    // For each horizontal bar(group), check if vertical bar(group) exists
    for (int b = 0; b < horiz_bars.size(); b++) {
        Point3i bar = horiz_bars[b];
        vector<Point3i> vert_bars;
        vert_bars.push_back(bar);
        bool found = false;
        
        int range = bar.z*2;
        // we only search in area around bar
        for (int c = max(0, bar.x-range); c < min(size.width, bar.x+range); c++) {
            Mat rMat = img.col(c).rowRange(max(0, bar.y-range), min(size.height, bar.y+range));
            if (_searchNestedCircles(rMat, c, vert_bars, false)) {
                found = true;
            }
        }
        if (found) {
            circles.push_back(vert_bars.at(0));
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
bool CircleMarker::_searchNestedCircles(Mat & row, int r, vector<Point3i> & bars, bool horizontal) {
    bool res = false;
    float max_ratio = 0.72f;
    uchar marker_flips = 8;

    int end = horizontal ? row.size().width : row.size().height;

    uchar flips = 0;
    uchar state = 1;
    ushort possible_start = 0;
    ushort definite_end = 0;
    ushort curr = 0;
    ushort prev = 0;

    bool black = (uchar)row.at<uchar>(0, 0) == 255;
    bool white = !black;

    for (int i = 0; i < end; i++) {
        bool next_white = horizontal ? (uchar)row.at<uchar>(0, i) == 0 : (uchar)row.at<uchar>(i, 0) == 0;

        if (next_white == white) curr++;
        white = next_white;
        black = !white;

        float ratio = max(curr, prev)/(float)(curr+prev);

        if (state == 1) { // Read "leading" blacks
            if (white) {
                state = 2;
                // cout << "2A";
                prev = curr;
                curr = 1;
                possible_start = i;
            }
        } else if (state == 2) { // Read white ring
            if (black) {
                if (curr < prev) {
                    state = 3;
                    // cout << "3A";
                    flips = 1;
                } else {
                    state = 1;
                    // cout << "1A";
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
                    // cout << "4A";
                    flips++;
                } else {
                    state = 2;
                    // cout << "2B";
                }
            } else if (curr > prev && ratio >= max_ratio) {
                state = 1;
                // cout << "1B";
            }
        } else if (state == 4) { // Read white ring
            if (black) {
                definite_end = i-1;

                prev = curr;
                curr = 1;
                if (ratio < max_ratio) {
                    if (flips == marker_flips) {
                        res = true;
                        int size = (definite_end - possible_start)/2;
                        int center = possible_start+size;
                        Point3i current(horizontal ? center : r, horizontal ? r : center, size);
                        bool found = false;
                        // cout << "|C: " << current.x << "," << current.y << "|" << endl;
                        for (int b = 0; b < bars.size(); b++) {
                            Point3i * other = &bars[b];
                            if (abs(current.x - other->x) < current.z && abs(current.y - other->y) < other->z) {
                                other->x = (current.x + other->x)/2;
                                other->y = (current.y + other->y)/2;
                                other->z = max(current.z, other->z);
                                found = true;
                                break;
                            }
                        }
                        if (!found) bars.push_back(current);
                    } else {
                        state = 3;
                        // cout << "3B";
                        flips++;
                    }
                } else {
                    state = 1;
                    // cout << "1C";
                }
            } else if (curr > prev && ratio >= max_ratio) {
                state = 2;
                //cout << "2C";
            }
        }
        //if (curr == 1)
        //    cout << (white ? " W " : " B ");
    }
    return res;
}
