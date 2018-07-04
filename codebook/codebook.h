#ifndef CODEBOOK_H
#define CODEBOOK_H

#endif // CODEBOOK_H

#include <stdio.h>
#include <stdlib.h>

#define uchar unsigned char

typedef struct ce{
    uchar learnHigh[1];
    uchar learnLow[1];
    uchar max[1];
    uchar min[1];
    int t_last_updata;
    int stale;
}code_element;

typedef struct code_book{
    int numEntries;
    int t;
    code_element cb[32];
}codeBook;

int makebm(uchar *p,codeBook *c,int size,int cbBounds,int numChannels);
int cleanbm(codeBook *c,int size);
int diffbm(uchar *p,codeBook *c,uchar *b,int size,int numChannels,int max,int min);

//void hotmap(uchar *im,int w,int h);
int hotmap(int *xx,int *yy,uchar *im,int w,int h);
//void codebook(uchar *im,int w,int h,codeBook *c,int flag,uchar *om);
int codebook(uchar *im,int w,int h,codeBook *c,int flag,uchar *om, int *xx, int *yy);
