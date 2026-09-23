#include <iostream>
#include  <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <vector>
using namespace cv;
using namespace std;
int main()
{
	string inputPath="resources/test_image.jpg";
	string outputDir = "result/task1_images";
	filesystem::create_directories(outputDir);
	Mat img =imread(inputPath);
	if(img.empty()){
	cerr <<"Cannot read image:"<<inputPath<<endl;
	return 1;
	}
	cout <<"Image loaded succeessfully."<<endl;
	cout<<"Width ="<<img.cols<<endl;
	cout<<"Height ="<<img.rows<<endl;
	Mat gray;
	cvtColor(img,gray,COLOR_BGR2GRAY);
	imwrite(outputDir +"/gray.png",gray);
	Mat hsv;
	cvtColor(img,hsv,COLOR_BGR2HSV);
	vector<Mat>hsvChannels;
	split(hsv,hsvChannels);
	imwrite(outputDir +"/hsv_h.png",hsvChannels[0]);
	imwrite(outputDir +"/hsv_s.png",hsvChannels[1]);
	imwrite(outputDir +"/hsv_v.png",hsvChannels[2]);
	Mat meanImg;
	Mat gaussianImg;
	Mat medianImg;
	blur(img,meanImg,Size(5,5));
	GaussianBlur(
	img,
	gaussianImg,
	Size(5,5),
	1.5
	);
	medianBlur(img,medianImg,5);
	imwrite(outputDir + "/mean_filter.png", meanImg);
        imwrite(outputDir + "/gaussian_filter.png", gaussianImg);
        imwrite(outputDir + "/median_filter.png", medianImg);
	Mat maskLow;
	Mat maskHigh;
	Mat redMask;
	inRange(
	hsv,
	Scalar(0,100,100),
	Scalar(18,255,255),
	maskLow
	);
	inRange(
        hsv,
        Scalar(170, 100, 100),
        Scalar(179, 255, 255),
        maskHigh
    );
	bitwise_or(maskLow,maskHigh,redMask);
	imwrite(outputDir + "/red_mask.png",redMask);
	Mat kernel =getStructuringElement(
	MORPH_RECT,
	Size(5,5)
	);
	Mat eroded;
	Mat dilated;
	Mat opened;
	Mat closed;
	erode(redMask,eroded,kernel);
	dilate(redMask,dilated,kernel);
	morphologyEx(
	redMask,
	opened,
	MORPH_OPEN,
	kernel
	);
	morphologyEx(
        redMask,
        closed,
        MORPH_CLOSE,
        kernel
    );
	imwrite(outputDir + "/erode.png", eroded);
        imwrite(outputDir + "/dilate.png", dilated);
        imwrite(outputDir + "/open.png", opened);
        imwrite(outputDir + "/close.png", closed);
	vector<vector<Point>>contours;
	Mat contourInput=closed.clone();
	findContours(
	contourInput,
	contours,
	RETR_EXTERNAL,
	CHAIN_APPROX_SIMPLE
	);
	Mat contourResult =img.clone();
	const double minArea =500.0;
	int validCount=0;
	for(size_t i=0;i<contours.size();++i){
	double area=contourArea(contours[i]);
	if(area<minArea){
	continue;
	}
	validCount++;
	drawContours(
	contourResult,
	contours,
	static_cast<int>(i),
	Scalar(0,255,0),
	2
	);
	Rect box =boundingRect(contours[i]);
	rectangle(
	contourResult,
	box,
	Scalar(0,0,255),
	2
	);
	string text ="Area=" + to_string(static_cast<int>(area));
	putText(
	   contourResult,
            text,
            Point(box.x, max(20, box.y - 5)),
            FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar(255, 0, 0),
            1
        );
	cout << "Contour "
             << validCount
             << ": area = "
             << area
             << endl;
    }

    imwrite(
        outputDir + "/contours_boxes.png",
        contourResult
    );
	Mat drawing =img.clone();
	Point center(
	img.cols /2,
	img.rows /2
	);
	circle(
	drawing,
	center,
	80,
	Scalar(255,0,0),
	3
	);
	rectangle(
    drawing,
    Point(50, 50),
    Point(250, 180),
    Scalar(0, 255, 0),
    3
);
	putText(
        drawing,
        "OpenCV Task 1",
        Point(50, 230),
        FONT_HERSHEY_SIMPLEX,
        1.0,
        Scalar(0, 0, 255),
        2
    );

    imwrite(
        outputDir + "/drawing.png",
        drawing
    );
	Point2f rotationCenter(
	img.cols /2.0f,
	img.rows /2.0f
	);
	Mat rotationMatrix =
		getRotationMatrix2D(
			rotationCenter,
			35.0,
			1.0
		);
	Mat rotated;
	warpAffine(
        img,
        rotated,
        rotationMatrix,
        img.size()
    );

    imwrite(
        outputDir + "/rotated_35deg.png",
        rotated
    );
	 Rect cropRegion(
        0,
        0,
        img.cols / 2,
        img.rows / 2
    );

    Mat cropped = img(cropRegion).clone();

    imwrite(
        outputDir + "/crop_top_left.png",
        cropped
    );


    cout << endl;
    cout << "Task 1 completed." << endl;
    cout << "Results saved to: "
         << outputDir
         << endl;

    return 0;
}
