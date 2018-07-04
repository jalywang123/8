#include "detection.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
//the edge detection based on wavelet modulus maxima
//extarct target area
Rect tagEdgeExtraction(Mat image, Rect tag_rect){

    int width = tag_rect.width;
    int height= tag_rect.height;
    int SIZE = 16;

    int i, j, k;
    //gadient x and y
    int Gx = 0, Gy = 0;

    //modulus value
    Mat Grad = Mat(height, width, CV_32FC1, float(0));
    //max modulus
    int max_Grad = 0;
    //target edge coordinate
    int minX = width, maxX = 0, minY = height, maxY = 0;
    //target image
    Mat tag_image;
    Rect rect;
    //gray image
    cvtColor(image, image, CV_RGB2GRAY);
    //target area
    tag_image = image(tag_rect);
    //calculate gadient x and y, modulus and max modulus
    for(i = 1; i < (height - 1); i++){
        for(j = 1; j < (width - 1); j++){
            k = 0;
            Gx = tag_image.at<uchar>(i + 1, j + 1) - tag_image.at<uchar>(i + 1, j);
            Gy = tag_image.at<uchar>(i + 1, j + 1) - tag_image.at<uchar>(i, j + 1);
            k = Gx * Gx + Gy * Gy;
            Grad.at<float>(i, j) = k;
            max_Grad = k > max_Grad ? k : max_Grad;
        }
    }
    //threshold
    max_Grad = 0.33 * max_Grad;
    //extract target area
    for(i = 1; i < (height - 1); i++){
        for(j = 1; j < (width - 1); j++){
            if(Grad.at<float>(i, j) >= max_Grad){
                minX = j > minX ? minX : j;
                minY = i > minY ? minY : i;
                maxX = j > maxX ? j : maxX;
                maxY = i > maxY ? i : maxY;
            }
        }
    }

    width = maxX - minX + 1;
    height = maxY - minY + 1;

    if(width < SIZE){
        minX -= (SIZE - width) / 2;
        width = (tag_rect.x + minX + SIZE) > 720 ? (720 - minX - tag_rect.x) :SIZE;
    }

    if(height < SIZE){
        minY -= (SIZE - height) / 2;
        height = (tag_rect.y + minY + SIZE) > 576 ? (576 - minY - tag_rect.y) :SIZE;
    }
    //calculate target area
    rect.x = tag_rect.x + minX;
    rect.y = tag_rect.y + minY;
    rect.width = width;
    rect.height = height;

    return rect;
}
