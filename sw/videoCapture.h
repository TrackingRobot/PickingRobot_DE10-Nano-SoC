#ifndef _VIDEOCAPTURE_H_
#define _VIDEOCAPTURE_H_

#include <opencv2\opencv.hpp>

#include "modify.h"
using namespace cv;

//int videoCap(VideoCapture &capture, float radius, float angle);
int videoCap(VideoCapture (&capture), _Point (&point)[MAX_SHUTTLECOCK], int numShuttlecock);

#endif