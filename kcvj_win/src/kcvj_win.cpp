#include<winsock2.h>
#include <Windows.h>

#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"


#pragma comment(lib,"ws2_32.lib")

#define SERVER "127.0.0.1"
#define BUFLEN 1024
#define PORT 9988
struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
char buf[BUFLEN];
char message[BUFLEN];
WSADATA wsa;


using namespace cv;
using namespace std;



Mat input, output;

void initNet() {
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0) {
        printf("Failed. Error Code : %d",WSAGetLastError());
        //exit(EXIT_FAILURE);
    }

    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
        printf("socket() failed with error code : %d" , WSAGetLastError());
        //exit(EXIT_FAILURE);
    }

    printf("Initialised.\n");
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
}

void send(string msg) {
	msg.copy(message, msg.length());

	if (sendto(s, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR) {
		printf("sendto() failed with error code : %d" , WSAGetLastError());
		//exit(EXIT_FAILURE);
	}
         
	//receive a reply and print it
	//clear the buffer by filling null, it might have previously received data
	memset(buf,'\0', BUFLEN);
}

int main(int argc, char* argv[]) {
    int camera_name = 0;
    string calibration = "logitech800x600.yml";
	int gui = 1;
    int thresh = 75;
    double markerSize = 25.0;

	initNet();
    
    Camera camera(calibration);
    VideoCapture cap(camera_name);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    } else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH, camera.width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
    }
    
    vector<CircleMarker> markers;
    markers.push_back(CircleMarker(0, markerSize));
    CircleMarker * marker0 = &(markers.at(0));

    
    cout << "Using camera " << camera_name << " with calibration file " << calibration;
    cout << " and gui " << (gui>0 ? "enabled" : "disabled") << endl;

    unsigned int frames_total = 0;
    int64 t0_total = GetTimeMs64();
    int64 t0 = GetTimeMs64();;
    
    while (true) {
        cap >> input;
        int t_grab = (int) (GetTimeMs64() - t0);

        t0 = GetTimeMs64();

        CircleMarker::findAndEstimate(input, output, gui>0, camera, markers, 0.45, thresh);
        int t_find = (int) (GetTimeMs64() - t0);

        if (marker0->detected) {
            cout << marker0->serialize() << endl;
			string res = marker0->serialize();
			send(res);
            marker0->detected = false;
        } else {
            cout <<  "T: - " << endl << "R: - " << endl;
        }
        
        if (gui > 0) {
            imshow("OUTPUT", output);
            if (waitKey(10) == 'q') break;
        }

        frames_total++;
        t0 = GetTimeMs64();
        cout << "Avg FPS:" << 1000.0/((unsigned int) ((t0-t0_total)/frames_total)) << ", ";
        cout << "Grab: " << t_grab << "ms, Find: " << t_find << "ms" << endl;
    }
    
    closesocket(s);
    WSACleanup();
    return 0;
}
