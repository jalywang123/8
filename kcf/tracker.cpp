//#include <algorithm>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

//#include "kcftracker.hpp"
//#include "tracker.h"

//#include <dirent.h>

//using namespace std;
//using namespace cv;

//bool HOG = true;
//bool FIXEDWINDOW = false;
//bool MULTISCALE = true;
//bool SILENT = true;
//bool LAB = false;

//// Create KCFTracker object
//KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

//Mat bef_tagImage;
//Rect tag_rect1, tag_rect2;
////track_num1:tracking start,target still
////track_num2:during tracking,target still
//int track_num1, track_num2;

////average similarity and average APCE
//float average_simi, average_APCE;

////KCF track
////Mat image: current frame image
////Rect rect: target area(called when the trace is entered)
////char track_flag: the logo(called when the trace is entered)
//char kcfTrack(cv::Mat image, cv::Rect &rect, char track_flag)
//{
//    //stable tracking result
//    Rect result;
//    char flag = 0;

//    //current target area
//    Mat cur_tagImage;

//    //before and current target similarity
//    float similarity = 0, APCE = 0, value = 0;

//    //stable tracking
//    if(track_flag == 1){
//        //calculate the current frame trace rerults
//        result = tracker.update(image);

//        if(result.x == 0XFFFF){
//            flag = 0;
//            //return 0;
//        }
//        else{

//            if((result.x + result.width) > T_WIDTH - 1){
//                result.width = T_WIDTH - 1 - result.x;
//            }
//            if((result.y + result.height) > T_HEIGHT - 1){
//                result.height = T_HEIGHT - 1 - result.y;
//            }

//            /************************************************************************************************
//            //current frame target area image
//            cur_tagImage = image(result);
//            //calculate response max response and APCE
//            APCE = tracker.tagAreaPeak(bef_tagImage, cur_tagImage, similarity);
//            //judgment value
//            value = APCE * similarity;

//            //replace target area
//            bef_tagImage = cur_tagImage;
//            rect = result;

//            //continue trackings:yes or no  //value < 12 || similarity < 0.6 //|| APCE > (2.5 * average_APCE)
//            if(average_APCE > (2.5 * APCE)){
//                flag = 0;
//                //return 0;//no
//            }
//            else{

//                if(result.x < 1 || result.y < 1 || result.x > T_WIDTH - 1 || result.y > T_HEIGHT - 1){
//                    flag = 0;
//                    //return 0;
//                }
//                else{
//                    //average similarity and APCE
//                    average_simi = (average_simi + similarity) / 2;
//                    average_APCE = (average_APCE + APCE) / 2;

//                    //draw rectangle
//                    rectangle( image, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 0, 255 ), 1, 8 );
//                    flag = 1;
//                    //return 1;//yes

//                    //tracking start,target still
//                    track_num1++;
//                    if(track_num1 > 50){
//                        if((abs(result.x - tag_rect1.x) < 3 && \
//                           abs(result.y - tag_rect1.y) < 3)){
//                            flag = 0;
//                            track_num1 = 0;
//                        }
//                    }
//                    //during tracking,target still
//                    if((abs(result.x - tag_rect2.x) <= 1 && \
//                       abs(result.y - tag_rect2.y) <= 1)){
//                        track_num2++;
//                        if(track_num2 > 100){
//                            flag = 0;
//                            track_num2 = 0;
//                        }
//                    }
//                    else{
//                        tag_rect2 = result;
//                        track_num2 = 0;
//                    }
//                }
//            }
//            ************************************************************************************************/
//            //draw rectangle
//            rectangle( image, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 0, 255 ), 1, 8 );
//            flag = 1;
//        }
//    }//enter tracking
//    else{
//        //tracking init
//        tracker.init(rect, image);
//        rectangle( image, Point( rect.x, rect.y ), Point( rect.x+rect.width, rect.y+rect.height), Scalar( 0, 255, 255 ), 1, 8 );
//        //save target area
//        bef_tagImage = image(rect);

//        average_simi = 0;
//        average_APCE = 0;

//        flag = 1;
//        //return 1;

//        track_num1 = 0;
//        track_num2 = 0;
//        tag_rect1 = rect;
//        tag_rect2 = rect;

//    }
//    return flag;
//}


#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "./kcf/kcftracker.hpp"

#include <dirent.h>

using namespace std;
using namespace cv;


bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;


// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

// Frame readed
cv::Mat frame;

// Tracker results
cv::Rect result;


float xMin,yMin,width,height;

// Frame counter
int nFrames = 0;

//Display framenum
char str[25];

// char KCFtrack_han(cv::Mat frame,char o,cv::Rect &tarLoc)
char kcfTrack(cv::Mat frame, cv::Rect &tarLoc, char track_flag)
{
    char flag = 0;
    // nFrames++;
    // printf("nFrames = %d,", nFrames);

    // Initation
    if(track_flag == 0)  // First frame: init
    {
        // First frame, given the target location
        xMin =(float)tarLoc.x;
        yMin =(float)tarLoc.y;
        width =(float)tarLoc.width;
        height =(float)tarLoc.height;

        // track parameters init
        //tracker.init( Rect(xMin, yMin, width, height), frame );
        tracker.init( tarLoc, frame );
        rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );

        flag = 1;
    }

    // Begin Tracking
    else
    {
        // printf("nFrames = %d,", nFrames);
        result = tracker.update(frame);
        rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 1, 8 );
        tarLoc = result;
    }


    // Track Fail : target occlusion , disappear , track error.
    if ( tracker.flag == 1 )  // track error
        flag = 0;
    else
        flag = 1;
    return flag;

}
void tune_coordinate(int *x0, int *x_range, const int &x_limit)
{
    // both sides overstep at x-axis
    if (*x_range >= x_limit)
    {
        *x_range = x_limit;
        *x0 = 0;
    }
    // left/top overstep
    else if (*x0 < 0)
    {
        *x0 = 0;
    }
    // right/bottom overstep
    else if (*x0+*x_range > x_limit)
    {
        *x0 = x_limit - *x_range;
    }
    // just in the range
    // else *x0 = *x0
}
