#ifndef WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#include <winsock2.h>
#include <Windows.h>
#pragma comment(lib,"ws2_32.lib")
WSADATA wsa;
#endif

#include <string>
#include <opencv2/opencv.hpp>
#include "CircleMarker.h"

using namespace cv;
using namespace std;

#define BUFLEN 1024
struct sockaddr_in si_other;
struct sockaddr_in si_me;
int s, slen=sizeof(si_other);
char buf[BUFLEN];
char message[BUFLEN];
int initNet();
void send(string msg);

#define SERVER "127.0.0.1"
#define PORT 9989
#define CAMERA 0
#define CALIBRATION "logitech800x600.yml"
#define GUI 1
#define THRESH 40
#define MARKER_SIZE 25.0
#define SEARCH_SCALE 0.6

Mat input, output;

int main(int argc, char* argv[]) {
	if (initNet() < 0) return -1;
    
	Camera camera(CALIBRATION);
    VideoCapture cap(CAMERA);
    if(!cap.isOpened()) {
        cout << "Failed to open capture device." << endl;
        return -1;
    } else {
        cap.set(CV_CAP_PROP_FRAME_WIDTH, camera.width);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, camera.height);
    }
    
    vector<CircleMarker> markers;
	markers.push_back(CircleMarker(0, MARKER_SIZE));
    CircleMarker * marker0 = &(markers.at(0));
    
	cout << "Using camera " << CAMERA << " with calibration file " << CALIBRATION;
    cout << " and gui " << (GUI > 0 ? "enabled" : "disabled") << endl;
    
    while (true) {
        cap >> input;
		if (input.empty()) continue; // Empty frame

        CircleMarker::findAndEstimate(input, output, GUI>0, camera, markers, SEARCH_SCALE, THRESH);

        if (marker0->detected) {
            cout << marker0->serialize() << endl;
			string res = marker0->serialize();
			send(res);
            marker0->detected = false;
        }
        
        if (GUI > 0) {
            imshow("OUTPUT", output);
            if (waitKey(10) == 'q') break;
        }
    }
    
#ifdef WIN32
    closesocket(s);
    WSACleanup();
#else
	close(s);
#endif

    return 0;
}

int initNet() {
    printf("\nInitialising Winsock...\n");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != NO_ERROR) {
        printf("Failed. Error Code : %d\n",WSAGetLastError());
		return -1;
    }

    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
        printf("socket() failed with error code : %d\n" , WSAGetLastError());
		return -1;
    }

    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);
    printf("Initialised.\n");
	return 1;
}

void send(string msg) {
	msg.copy(message, msg.length());
	if (sendto(s, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR) {
		printf("sendto() failed with error code : %d" , WSAGetLastError());
	}
	memset(buf,'\0', BUFLEN);
}