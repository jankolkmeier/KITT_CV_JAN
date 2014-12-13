#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
	int gui = 1;
    int thresh = 75;
    double searchScale = 0.4;
    
    vector<Camera *> calibrations;
    //calibrations.push_back(new Camera("macbook640x480.yml"));
    //calibrations.push_back(new Camera("philips800x600.yml"));
    //calibrations.push_back(new Camera("logitech800x600.yml"));
    calibrations.push_back(new Camera("logitech800x600.yml"));
    
    vector<Mat> inputs;
    vector<Mat> outputs;

    
    vector<VideoCapture *> cams;
    for (int c = 0; c < calibrations.size(); c++) {
        cout << "Opening camera " << c << "...";
        cams.push_back(new VideoCapture(c));
        if (!cams[c]->isOpened()) {
            cout << " ...Failed!" << c << endl;
            return -1;
        }
        
        cout << " ...Success!" << endl;
        cams[c]->set(CV_CAP_PROP_FRAME_WIDTH, calibrations[c]->width);
        cams[c]->set(CV_CAP_PROP_FRAME_HEIGHT, calibrations[c]->height);
        inputs.push_back(Mat());
        outputs.push_back(Mat());
        cout << "Set camera parameters. Getting test frames: ";
        int f = 0;
        while (f++ < 10) {
            cams[c]->read(inputs[c]);
            cout << ">";
        }
        cout << " Done." << endl;
    }
    cout << "All cameras opened. " << endl;
    
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, 25.0));
    
    while (true) {
        
        for (int c = 0; c < calibrations.size(); c++) {
            cams[c]->read(inputs[c]);
            CircleMarker::findAndEstimate(inputs[c], outputs[c], gui > 0, *calibrations[c], markers, searchScale, thresh);
            
            string windowName = format("OUTPUT_%d", c);
            if (gui > 0) imshow(windowName, outputs[c]);
        }
        
        for (int m = 0; m < markers.size(); m++) {
            if (markers.at(m).detected) {
                cout << markers.at(m).serialize() << endl;
                markers.at(m).detected = false;
            } else {
                //cout <<  " - " << endl;
            }
        }
        
        if (waitKey(30) == 'q') break;
    }
    
    return 0;
}
