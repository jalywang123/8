#ifndef VIBE_H
#define VIBE_H

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

#define NUM_SAMPLES 20
#define MIN_MATCHES 2
#define RADIUS 20
#define SUBSAMPLE_FACTOR 9//16

#define D_WIDTH 360//720
#define D_HEIGHT 288//576


class ViBe_BGS
{
public:
    ViBe_BGS(void);
    ~ViBe_BGS(void);

    void init(const Mat _image);
    void processFirstFrame(const Mat _image);
    void testAndUpdate(const Mat _image);
    Mat getMask(void){return m_mask;};

    //void deleteSamples(){delete samples;};

private:
//  unsigned char ***samples;
//  float samples[1024][1024][NUM_SAMPLES+1];

/*
    Mat m_samples[NUM_SAMPLES];
    Mat m_foregroundMatchCount;*/

    Mat m_mask;
};

void cluster(Mat binary_img);
int vibeDetect(Mat img, ViBe_BGS Vibe_Bgs, int count, int *xx, int *yy);

#endif // VIBE_H

