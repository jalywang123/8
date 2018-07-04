#include "dataAssociation.h"

using namespace std;
using namespace cv;

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int tophatDetect(Mat img, int *xx, int *yy)
{
    Mat erode_img, dilate_img, difference_img, binary_img;

    Mat erode_element(19, 19, CV_8U, Scalar(1));
    Mat dilate_element(19, 19, CV_8U, Scalar(0));

    double v_max, v_min;
    vector<Point> pointsNonZero;
    int pointNum = 0, i;

    dilate_element.col(0) = 1;
    dilate_element.col(18) = 1;
    dilate_element.row(0) = 1;
    dilate_element.row(18) = 1;

    dilate(img, dilate_img, dilate_element, Point(9, 9));
    erode(dilate_img, erode_img, erode_element, Point(9, 9));
    subtract(img, erode_img, difference_img);

    minMaxIdx(difference_img, &v_min, &v_max);

    if(v_max > 20)
    {
        threshold(difference_img, binary_img, 0, 255, CV_THRESH_OTSU);
        findNonZero(binary_img, pointsNonZero);

        for(i = 0; i < pointsNonZero.size(); i++)
        {
            xx[i] = pointsNonZero[i].x;
            yy[i] = pointsNonZero[i].y;
        }

        pointNum = pointsNonZero.size();
    }
    else
        pointNum = 0;

    return pointNum;
}
