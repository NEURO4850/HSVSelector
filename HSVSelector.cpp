#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>

using namespace cv;
using namespace std;

// Adapted from
// http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html

vector<Point> findBestContour(const Mat &thresh);
pair<double, double> getXY(Rect bRect, int area, int width, int height);

vector<double> xVals = vector<double>(10, 0);
vector<double> yVals = vector<double>(10, 0);

int main(int argc, char **argv) {
    VideoCapture cap(0); // capture the video from web cam

    if (!cap.isOpened()) // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }

    namedWindow("Control",
                CV_WINDOW_AUTOSIZE); // create a window called "Control"

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    // Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); // Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); // Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); // Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    while (true) {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

        if (!bSuccess) // if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        Mat imgHSV;

        cvtColor(imgOriginal, imgHSV,
                 COLOR_BGR2HSV); // Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV),
                Scalar(iHighH, iHighS, iHighV),
                imgThresholded); // Threshold the image

 

        Mat thresholded = imgThresholded.clone();
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(thresholded, contours, hierarchy, CV_RETR_TREE,
                     CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        double max_area = 0;
        vector<Point> best;
        for (unsigned int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                best = contours[i];
            }
        }

        if (best.size() > 0) {
            Rect bRect = boundingRect(best);
            pair<double, double> xy = getXY(bRect, contourArea(best),
                                            cap.get(CV_CAP_PROP_FRAME_WIDTH),
                                            cap.get(CV_CAP_PROP_FRAME_HEIGHT));
            circle(imgOriginal,
                   Point(xy.first + cap.get(CV_CAP_PROP_FRAME_WIDTH) / 2.0,
                         xy.second + cap.get(CV_CAP_PROP_FRAME_HEIGHT) / 2.0),
                   bRect.height / 2, Scalar(0, 255, 0), -1);
        }

     // show the thresholded image

                imshow("Thresholded Image",
               imgThresholded);    

        imshow("Original", imgOriginal); // show the original image

        std::cout << "Threshold Values: " << std::endl;
        std::cout << "Lower: H: " << iLowH << " S: " << iLowS << " V: " << iLowV << std::endl;
        std::cout << "Higher: H: " << iHighH << " S: " << iHighS << " V: " << iHighV << std::endl;


        if (waitKey(30) == 27) // wait for 'esc' key press for 30ms. If 'esc'
                               // key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;
}

vector<Point> findBestContour(const Mat &thresh) {
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(thresh, contours, hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    double max_area = 0;
    vector<Point> best;
    for (unsigned int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            best = contours[i];
        }
    }

    return best;
}

pair<double, double> getXY(Rect bRect, int area, int width, int height) {
    double cx = bRect.x + (bRect.width / 2);
    double cy = bRect.y + (bRect.height / 2);
    double XError = cx - width / 2.0;
    double YError = cy - height / 2.0;

    if (area > 1000) {
        xVals.erase(xVals.begin());
        xVals.push_back(XError);
        yVals.erase(yVals.begin());
        yVals.push_back(YError);
    }

    double x = accumulate(xVals.begin(), xVals.end(), 0.0) / xVals.size();
    double y = accumulate(yVals.begin(), yVals.end(), 0.0) / yVals.size();

    return make_pair(x, y);
}
