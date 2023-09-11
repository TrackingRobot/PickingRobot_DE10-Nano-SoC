#include "videoCapture.h"

#define BAR1NAME "size_v"
#define BAR2NAME "size_h"

#define WINDOW1NAME "Badminton recognition."
#define WINDOW2NAME "��ֵ��"
#define WINDOW3NAME "outline"

#define WIDEANGLE_START 76
#define WIDEANGLE_END   122
#define MIDDLE_ANGLE    98.94

//���Ĳ������к궨��
#define PARA_A  416.622303830546
#define PARA_B  1.87028747527089
#define PARA_C  315.966144666733
#define PARA_D  47.3964798918816

//��ʾ�ķ�����
#define NUM_RECT 3
static int sizeOfSelect_v=150;
static int sizeOfSelect_h=200;

static int _minh = 40;
static int _maxh = 160;
static int _mins = 3;
static int _maxs = 50;
static int _minv = 150;
static int _maxv = 250;

/*
�ײ���[(236@79),(238@116)],��ë���С197*280,191*282    ---237  290*290 
�в���[(1014@82),(1015@121],��ë���С67*82��60*83		---1014 90*90
������[(1692@82),(1727@122)],��ë���С42*54,39*54		---1708 60*60

�Ƕȶ�Ϊ82--120�����Ƕ�λ��Ϊ 
��������λ�ã�
(�Ƕ�ֵ-82)*640/(120-82)
��ѡ���С��
��Ϸ���ʽ��Y = (A - D) / [1 + (X/C)^B] + D
������
A = 290.087087091475
B = 6.70452998764013
C = 768.104581633397
D = 58.916153673649
*/

void on_TrackBar( int, void *);

int videoCap(VideoCapture &capture,_Point (&point)[MAX_SHUTTLECOCK],int numShuttlecock )
{

	Mat edge, frame, fhsv[5],colorRecognition[5],grayPicture;
	
	int i, j, k;

	int *minh = &_minh;
	int *maxh = &_maxh;
	int *mins = &_mins;
	int *maxs = &_maxs;
	int *minv = &_minv;
	int *maxv = &_maxv;
	
	int* sizeOfSlect_V = &sizeOfSelect_v;
	int* sizeOfSlect_H = &sizeOfSelect_h;

	int position_X[NUM_RECT] = { 0 }, position_Y[NUM_RECT] = { 0 }, size_X[NUM_RECT] = { 0 }, size_Y[NUM_RECT] = { 0 };
	int point_x1[NUM_RECT], point_y1[NUM_RECT];// point_x2, point_y2;
	//int* min = &sizeOfSelect_v;
	//int* sizeOfSlect_H = &sizeOfSelect_h;

	Rect rect[NUM_RECT];
	Mat Candidate[NUM_RECT];
	Mat imgThreshold, outLine[NUM_RECT];

	char Trackbar1Name[50];
	char Trackbar2Name[50];
	char candidate[20] = "slect";
	char str[2] = { 0 };

	sprintf(Trackbar1Name, "size_v%3d", BAR1NAME);
	sprintf(Trackbar2Name, "size_h%3d", BAR2NAME);

	capture >> frame;
	namedWindow(WINDOW1NAME, 1);

	//��ѡ������(position_X,position_Y)����
	for (i = 0; i<NUM_RECT; i++)
	{

		if (point[i].angle <= WIDEANGLE_END && point[i].angle >= WIDEANGLE_START && point[i].radius > 0)
		{
			if (point[i].angle >= MIDDLE_ANGLE)
				position_X[i] = (frame.cols / 2 - tan((point[i].angle - MIDDLE_ANGLE) / 180 * 3.14) / tan((WIDEANGLE_END - MIDDLE_ANGLE  ) / 180 * 3.14)  * frame.cols / 2);
			else
				position_X[i] = (frame.cols / 2 + tan((MIDDLE_ANGLE - point[i].angle) / 180 * 3.14) / tan((MIDDLE_ANGLE - WIDEANGLE_START) / 180 * 3.14)  * frame.cols / 2);
		}
		position_Y[i] = 69.1196401034305 + 94655.6571285371 / point[i].radius;
	
		//��ѡ���С����
		size_Y[i] = 23.0431065523619 + 60594.1611566076 / point[i].radius;
		size_Y[i] *= 1.0;
		size_X[i] = size_Y[i];

		//��ѡ�򶥵�
		point_x1[i] = position_X[i] - size_X[i] / 2;
		point_y1[i] = position_Y[i] - size_Y[i] * 1.5 / 2;

		//����[(0,0),(640,480)]����
		if (point_x1[i] < 0 || point_x1[i] > frame.cols - 1)
			point_x1[i] = 0;
		if (point_y1[i] < 0 || point_y1[i] > frame.rows - 1)
			point_y1[i] = 0;
		//if (point_x2 > 640-1)
		//	point_x2 = 640-1;
		//if (point_y2 < 480-1)
		//	point_y2 = 480-1;

		if (point_x1[i] + size_X[i] > 640 - 1)//�ҳ���
			size_X[i] = frame.cols - 1 - point_x1[i];
		if (point_y1[i] + size_Y[i] > 480 - 1)
			size_Y[i] = frame.rows - 1 - point_y1[i];
		//������ѡ��
		if (point_x1[i] + size_X[i] > point_x1[i])
		rect[i] = ( Rect(point_x1[i], point_y1[i], size_X[i], size_Y[i]) );//cv::Rect rect(x, y, width, height);

		//������ѡ����
		Candidate[i] = (Mat(frame, rect[i]));// Rect rect(point_x1, point_y1, *sizeOfSlect_H, *sizeOfSlect_V);

		sprintf(str, "%1d", i);
		strcat(candidate, str);
		imshow(candidate,Candidate[i]);

		//ת��ΪHSV��ʽ
		cvtColor(Candidate[i], fhsv[i], COLOR_BGR2HSV);

		Mat imgThreshold;
		inRange(fhsv[i], Scalar(*minh, *mins, *minv), Scalar(*maxh, *maxs, *maxv), imgThreshold);
		//������ (ȥ��һЩ���)  �����ֵ����ͼƬ���Ų�����Ȼ�ܶ࣬���������size
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(imgThreshold, imgThreshold, MORPH_OPEN, element);

		//�ղ���
		morphologyEx(imgThreshold, imgThreshold, MORPH_CLOSE, element);
		//�ԻҶ�ͼ�����˲�
		GaussianBlur(imgThreshold, imgThreshold, Size(3, 3), 0, 0);

		//namedWindow(WINDOW2NAME, 1);
		//�����������ڵ�����ѡ���С
		//createTrackbar(Trackbar1Name, WINDOW2NAME, sizeOfSlect_V, frame.cols, on_TrackBar);
		//createTrackbar(Trackbar2Name, WINDOW2NAME, sizeOfSlect_H, frame.rows, on_TrackBar);

		//��ɫʶ����ֵ��
		inRange(fhsv[i], Scalar(*minh, *mins, *minv), Scalar(*maxh, *maxs, *maxv), colorRecognition[i]);//void rectangle(Mat& img, Point pt1,Point pt2,const Scalar& color, int thickness=1, int lineType=8, int shift=0)
		imshow(candidate, colorRecognition[i]);


		printf("(R%.0f@%.0f) ", point[i].radius, point[i].angle);
		//��Ե���
		Canny(colorRecognition[i], outLine[i], 100, 200, 3, false);
		//namedWindow(WINDOW3NAME, 1);
		//imshow(WINDOW3NAME, outLine[i]);

		//�Խ���ж�
		CvPoint pointHSV;
		int countH, countS, countV, countHSV = 0, totalColorRecognition = 0;
		//pointHSV=colorRecognition.
		for (k = colorRecognition[i].cols*0.1; k < colorRecognition[i].cols*0.8; k++)
		{
			for (j = colorRecognition[i].rows*0.1; j < colorRecognition[i].rows*0.8; j++)
			{
				if (
					//(fhsv.at<Vec3b>(Point(pointHSV))[0] > (*minh) && fhsv.at<Vec3b>(Point(pointHSV))[0] < (*maxh) ) &&
					//(fhsv.at<Vec3b>(Point(pointHSV))[1] > (*mins) && fhsv.at<Vec3b>(Point(pointHSV))[1] < (*maxs) ) &&
					//(fhsv.at<Vec3b>(Point(pointHSV))[2] > (*minv) && fhsv.at<Vec3b>(Point(pointHSV))[2] < (*maxv) )
					colorRecognition[i].at<uchar>(k, j) == 255
					)
					countHSV++;
				//printf("%d ", colorRecognition.at<uchar>(i, j));
				//waitKey(10);
			}
		}
		totalColorRecognition = k*j;

		if (countHSV >= totalColorRecognition*0.5)
			return 1;
			//rectangle(frame, rect[i], Scalar(0, 255, 0), 3, 8, 0);
		else
			return 0;
			//rectangle(frame, rect[i], Scalar(255, 0, 0), 2, 8, 0);

	}

	


	//hsv�����������ڲ�����ë�����
	//createTrackbar("minh", WINDOW1NAME, minh, 180, on_TrackBar);
	//createTrackbar("maxh", WINDOW1NAME, maxh, 180, on_TrackBar);

	//createTrackbar("mins", WINDOW1NAME, mins, 255, on_TrackBar);
	//createTrackbar("maxs", WINDOW1NAME, maxs, 255, on_TrackBar);

	//createTrackbar("minv", WINDOW1NAME, minv, 255, on_TrackBar);
	//createTrackbar("maxv", WINDOW1NAME, maxv, 255, on_TrackBar);

	//������ѡ��
	//Rect rect(point_x1, point_y1, size_X, size_Y);//cv::Rect rect(x, y, width, height);



	//printf("[��%d*%d)@(H:%d)]rectangle<(%d,%d),(%d,%d)>\n",frame.rows,frame.cols, position_X, point_x1, point_y1, point_x2, point_y2);





	////�Խ���ж�
	//CvPoint pointHSV;
	//int countH, countS, countV, countHSV = 0, totalColorRecognition = 0;
	////pointHSV=colorRecognition.
	//for (i = 10; i < colorRecognition.cols*0.9; i++)
	//{
	//	for (j = 10; j < colorRecognition.rows*0.9; j++)
	//	{
	//		if (
	//			//(fhsv.at<Vec3b>(Point(pointHSV))[0] > (*minh) && fhsv.at<Vec3b>(Point(pointHSV))[0] < (*maxh) ) &&
	//			//(fhsv.at<Vec3b>(Point(pointHSV))[1] > (*mins) && fhsv.at<Vec3b>(Point(pointHSV))[1] < (*maxs) ) &&
	//			//(fhsv.at<Vec3b>(Point(pointHSV))[2] > (*minv) && fhsv.at<Vec3b>(Point(pointHSV))[2] < (*maxv) )
	//			colorRecognition.at<uchar>(i, j) == 255
	//			)
	//			countHSV++;
	//			//printf("%d ", colorRecognition.at<uchar>(i, j));
	//			//waitKey(10);
	//	}
	//}
	//totalColorRecognition = i*j;

	//if (countHSV >= totalColorRecognition*0.2)
	//	rectangle(frame, rect, Scalar(0, 255, 0), 3, 8, 0);
	//else
	//	rectangle(frame, rect, Scalar(255, 0, 0), 2, 8, 0);
	
	imshow(WINDOW1NAME, frame);

	waitKey(20);
	return 0;
}


void on_TrackBar(int, void *)
{
	;
}
/*
ͼ��������������Mat D(A,rect(start_h,start_v,end_h,end_v))����Mat E=A(range(start_h,start_end),range(end_h,end_v) )����һ������Ȥ������

ת��Iplimage*ΪMat Mat mtx(img);
*/