#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

// #include "track.hpp"

#define LINT_ARGS 1
extern "C"
{
    #include "libyolo.h"
}

const int SUB_RECT_WIDTH = 720;
const int SUB_RECT_HEIGHT = 576;

struct Region
{
    int x, y;
    int width = SUB_RECT_WIDTH;
    int height = SUB_RECT_HEIGHT;
};

using namespace std;
using namespace cv;

class Detection
{
    public:
        Detection(char *darknet_path, char *datacfg, char *cfgfile, char *weightfile);
        ~Detection();

        // detect with image object in memory
        box detect_image(Mat frame, double thresh, double hier_thresh);
        // detect with disk frame file
        box detect_image(Mat frame, char *frame_file, double thresh, double hier_thresh);

        // detect schedule
        box detect(Mat frame, char *frame_file, double thresh, double hier_thresh,
                   int obj_track_x, int obj_track_y);

        //detect multiple target
        int detect_image(Mat frame, char *frame_file, double thresh, double hier_thresh, box* obj_rect);

        // convert a box from a sub-region to the whole range
        void subox_to_objbox();
        // convert a box from the whole range to a sub-region
        void objbox_to_subox();
        
        // getter
        box get_obj_box() const;
        box get_sub_box() const;
        Region get_sub_rect() const;


    protected:


    private:
        void ipl_into_image(IplImage* src, image im);
        image ipl_to_image(IplImage* src);
        void new_subrect(int cx, int cy, int cols, int rows);
        box select_obj_box(detection_info **info, int num);
        void av(Mat tagimg, double average, double variance);
        double  ncc_similarity(Mat origintag,Mat tag1,double originaverage, double originvariance, double average, double variance);

        box obj_box, sub_box;
        Region sub_rect;
        yolo_handle yolo;
};

Rect tagEdgeExtraction(Mat image, Rect tag_rect);

#endif // DETECTION_H

