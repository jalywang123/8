#include "vibe.h"
#include <cstdio>

using namespace cv;
using namespace std;

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void cluster(Mat binary_img)
{
    int i, j, k, m, n, l;
    Mat mat, cluster_img, trans_img;
    int count = 0, max_count = 0;
    vector<Point> pointsNonZero;

    double sum_value = 0, mean_value = 0;

    cluster_img = Mat::zeros(binary_img.size(),CV_32FC1);

    findNonZero(binary_img, pointsNonZero);

    if(pointsNonZero.size() < 5000)
    {
        for(k = 0; k < pointsNonZero.size(); k++)
        {
            if(pointsNonZero[k].y < 10 || pointsNonZero[k].y > binary_img.rows - 10 || \
               pointsNonZero[k].x < 10 || pointsNonZero[k].x > binary_img.cols - 10)
                continue;
            for(i = pointsNonZero[k].y - 3; i < pointsNonZero[k].y + 3; i++)
            {
                for(j = pointsNonZero[k].x - 3; j < pointsNonZero[k].x + 3; j++)
                {
                    count = 0;
                    for(m = -3; m < 3; m++)
                    {
                        for(n = -3; n < 3; n++)
                        {
                            if(binary_img.at<uchar>(i + m, j + n) == 255)
                                count++;
                        }
                    }
                    cluster_img.at<float>(i, j) = count;
                    max_count = count >= max_count ? count : max_count;
                 }
            }
        }

        normalize(cluster_img, cluster_img, 1.0, 0.0, NORM_MINMAX);
        cluster_img.convertTo(binary_img, CV_8UC1, 255, 0);
        threshold(binary_img, binary_img, 120, 255, CV_THRESH_BINARY);
    }
}

int vibeDetect(Mat img, ViBe_BGS Vibe_Bgs, int count, int *xx, int *yy)
{
    Mat binary_img;
    double v_max, v_min;
    vector<Point> pointsNonZero;
    int pointNum = 0, i;

    Mat erode_element(5, 1, CV_8U, Scalar(1));
    Mat dilate_element(5, 5, CV_8U, Scalar(1));
    Mat erode_element1(3, 3, CV_8U, Scalar(1));

//    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5), Point(2, 2));

    if (count == 1)
    {
        Vibe_Bgs.init(img);
        Vibe_Bgs.processFirstFrame(img);
    }
    else
    {
        Vibe_Bgs.testAndUpdate(img);
        binary_img = Vibe_Bgs.getMask();
    //  imshow("binary_img1", binary_img);
    //  cvWaitKey(1);

 //       morphologyEx(binary_img, binary_img, MORPH_OPEN, element);
        erode(binary_img, binary_img, erode_element, Point(0, 2));
       // imshow("binary_img2", binary_img);
       // cvWaitKey(1);
   //     dilate(binary_img, binary_img, dilate_element, Point(2, 2));
  //      erode(binary_img, binary_img, erode_element1, Point(1, 1));
       // imshow("binary_img2", binary_img);
       // cvWaitKey(1);
        cluster(binary_img);
//       imshow("binary_img2", binary_img);
 //     cvWaitKey(1);


 //        threshold(binary_img, binary_img, 0, 255, CV_THRESH_OTSU);


        findNonZero(binary_img, pointsNonZero);

        if(pointsNonZero.size() < 5000)
        {
            for(i = 0; i < pointsNonZero.size(); i++)
            {
                if(pointsNonZero[i].x > 10 && pointsNonZero[i].x < (img.cols - 10) && \
                   pointsNonZero[i].y > 10 && pointsNonZero[i].y < (img.rows - 10))
                {
                    xx[i] = pointsNonZero[i].x;
                    yy[i] = pointsNonZero[i].y;
                    pointNum++;
                }
            }
        }
    }
    return pointNum;
}
