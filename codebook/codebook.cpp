#include "codebook.h"



int cvupdataCodeBook(uchar* p,codeBook &c,int cbBounds,int numChannels)
{
    if(c.numEntries==0) c.t=0;
    c.t+=1;
    int n;
    int high[3],low[3];
    for (n=0;n<numChannels;n++)
    {
        high[n]=*(p+n)+cbBounds;
        if(high[n]>255) high[n]=255;
        low[n]=*(p+n)-cbBounds;
        if(low[n]<0)
            low[n]=0;
    }
    int matchChannel;
    int i;

    for(i=0;i<c.numEntries;i++)
    {
        matchChannel=0;
        for(n=0;n<numChannels;n++)
        {
            if((c.cb[i].learnLow[n]<=*(p+n))&&(*(p+n)<=c.cb[i].learnHigh[n]))
                matchChannel++;
        }
        if(matchChannel==numChannels)
        {

            c.cb[i].t_last_updata=c.t;
            for(n=0;n<numChannels;n++)
            {
                if(c.cb[i].max[n]<*(p+n))
                    c.cb[i].max[n]=*(p+n);
                else if(c.cb[i].min[n]>*(p+n))
                    c.cb[i].min[n]=*(p+n);
            }
            break;
        }
    }

    if(i==c.numEntries)
    {
        for(n=0;n<numChannels;n++)
        {
            c.cb[c.numEntries].learnHigh[n]=high[n];
            c.cb[c.numEntries].learnLow[n]=low[n];
            c.cb[c.numEntries].max[n]=*(p+n);
            c.cb[c.numEntries].min[n]=*(p+n);
        }
        c.cb[c.numEntries].t_last_updata=c.t;
        c.cb[c.numEntries].stale=0;
        c.numEntries+=1;
    }

    for(int s=0;s<c.numEntries;s++)
    {
        int negRun=c.t-c.cb[s].t_last_updata;
        if(c.cb[s].stale<negRun)
            c.cb[s].stale=negRun;
    }

    for(n=0;n<numChannels;n++)
    {
        if(c.cb[i].learnHigh[n]<high[n])
            c.cb[i].learnHigh[n]+=1;
        if(c.cb[i].learnLow[n]>low[n])
            c.cb[i].learnLow[n]+=1;
    }

    return i;
}

uchar cvbackgroundDiff(uchar *p,codeBook &c,int numChannels,int minMod,int maxMod)
{
    int matchChannel;
    int i,n;
    for(i=0;i<c.numEntries;i++)
    {
        matchChannel=0;
        for(n=0;n<numChannels;n++)
        {
            if((c.cb[i].min[n]-minMod<=*(p+n))&&(*(p+n)<=c.cb[i].max[n]+maxMod))
                matchChannel++;
            else
                break;
        }
        if(numChannels==matchChannel)
            break;
    }
    if(i==c.numEntries)
        return 255;
    return 0;
}

int cvlearStaleEntries(codeBook &c)
{
    int staleThresh = c.t>>1;
    int *keep=new int [c.numEntries];
    int keepCnt=0;

    for(int i=0;i<c.numEntries;i++)
    {
        if(c.cb[i].stale>staleThresh)
            keep[i]=0;
        else
        {
            keep[i]=1;
            keepCnt+=1;
        }
    }

    c.t=0;
    code_element foo[128];
    int k=0;
    for(int ii=0;ii<c.numEntries;ii++)
    {
        if(keep[ii])
        {
            foo[k]=c.cb[ii];
            foo[k].stale=0;
            foo[k].t_last_updata=0;
            k++;
        }
    }

    delete [] keep;
    for(int ii=0;ii<c.numEntries;ii++)
        c.cb[ii]=foo[ii];

    int numCleared=c.numEntries-keepCnt;
    c.numEntries=keepCnt;

    return numCleared;
}

int makebm(uchar *p,codeBook *c,int size,int cbBounds,int numChannels)
{
    int i;
    //uchar *a=(uchar*)calloc(3,sizeof(uchar));
    for(i=0;i<size;i++)
    {
       // a[0]=(uchar)(p[i*3+2]*0.3+0.59*p[i*3+1]+0.11*p[i*3]);
        cvupdataCodeBook(p+i,c[i],cbBounds,numChannels);
    }
   // free(a);
    return 1;
}

int cleanbm(codeBook *c,int size)
{
    int i;
    for(i=0;i<size;i++)
    {
        cvlearStaleEntries(c[i]);
    }
    return 1;
}

int diffbm(uchar *p,codeBook *c,uchar *b,int size,int numChannels,int max,int min)
{
    int i;
   // uchar *a=(uchar*)calloc(3,sizeof(uchar));
    for(i=0;i<size;i++)
    {
        //a[0]=(uchar)(p[i*3+2]*0.3+0.59*p[i*3+1]+0.11*p[i*3]);
        b[i]=cvbackgroundDiff(p+i,c[i],numChannels,max,min);
       // b[3*i+1]=b[3*i+2]=b[3*i];
    }
   // free(a);
    return 1;

}

void guiyi(float *a,int s)
{
    float max=0;int i;
    for(i=0;i<s;i++)
        if(a[i]>max) max=a[i];
    for(i=0;i<s;i++)
        a[i]=a[i]/max*100;
    for(i=0;i<s;i++)
    {
        if(a[i]>60) a[i]=255;
        else a[i]=0;
    }
}

int hotmap(int *xx,int *yy,uchar *im,int w,int h)
{
    int i,j,k,l;
    float *map=(float*)calloc(w*h,sizeof(float));

    int num = 0;

    for(i=5;i<h-5;i++)
        for(j=5;j<w-5;j++)
        {
           for(k=-5;k<5;k++)
               for(l=-5;l<5;l++)
               {
                   if(im[(i+k)*w+(j+l)]==255)

                     map[i*w+j]++;
               }
        }
    guiyi(map,w*h);
//    for(i=0;i<w*h;i++)
//        im[i]=map[i];
    for(i=0;i<h;i++)
    {
        for(j=0;j<w;j++)
        {
            if(map[i*w+j]==255)
            {
                xx[num]=j;
                yy[num]=i;
                num++;
            }
        }
    }
    return num;
}

int codebook(uchar *im,int w,int h,codeBook *c,int flag,uchar *om, int *xx, int *yy)
{
    int fnum=50;
    int size=w*h;

    int pointNum = 0;

    if(flag<fnum)
        makebm(im,c,size,5,1);
    else if(flag==fnum)
        cleanbm(c,size);
    else
    {
        diffbm(im,c,om,size,1,20,20);
        pointNum = hotmap(xx,yy,om,w,h);
    }
    return pointNum;
}
