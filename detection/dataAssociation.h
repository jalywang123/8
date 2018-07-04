#ifndef DATAASSOCIATION_H
#define DATAASSOCIATION_H

#include <iostream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "./darknet/include/darknet.h"

#define F 15
#define MaxBlockSize 2500
#define MinBlockSize 9

using namespace std;
using namespace cv;

typedef struct
{
    box rect;//target left-top coordinate,width,height
    int intNum;//interconnection num
    char holdNum;//keep num
    char markFlag;//target mark
    int IP;//target label
}AREA;

//Top-Hat detection
int tophatDetect(Mat img, int *xx, int *yy);
//get area
int getarea(int *xx, int *yy, int *num, int width, int height, AREA *out);
//init data
void initData(AREA *curTag, AREA *befTag, char *tagStatus);
//data coversion
int dataConversion(AREA* obj_rect, int obj_num, AREA *curTag);
//data association
int dataAssociation(AREA *bef, int befNum, AREA *cur, int curNum, AREA *obj_rect, char detect_flag);


#endif // DATAASSOCIATION_H

