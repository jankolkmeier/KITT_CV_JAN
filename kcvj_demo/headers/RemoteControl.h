#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <iostream>

#ifndef WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <pthread.h>
#include <unistd.h>
#endif

#include <opencv2/opencv.hpp>

class Settings;

class RemoteControl {
    static void *listen(void *arg);
public:
    bool running;
    std::string settingsFile;
    RemoteControl(int, std::string);
    void setDebugImage(cv::Mat &img);
    void die(std::string, int);

    bool imageSet;
    
#ifndef WIN32
    pthread_t thread;
    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    struct sockaddr_in si_me;
    struct sockaddr_in si_other;
    socklen_t slen;
    int listenSock;
    int image_requested;
#endif

    Settings * settings;

    cv::Mat sendFrame;
    int width;
    int height;
    int elemSize;
    int imgSize;
};
