#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <QString>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "./detection/detection.h"
#include "./kcf/tracker.h"
#include "./detection/dataAssociation.h"
#include "./codebook/codebook.h"
#include "./vibe/vibe.h"
#include "socket.h"
#include <dirent.h>

#include "./svm/svm.hpp"

using namespace std;
using namespace cv;

char tagStatus[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int main()
{
    char *darknet_path = "./darknet";
    char *datacfg = "model/tiny-yolo_oneClass/voc.data";
    char *cfgfile = "model/tiny-yolo_oneClass/tiny-yolo.cfg";
    char *weightfile = "model/tiny-yolo_oneClass/tiny-yolo.backup";
    double thresh = 0.25;//UAV 0.25
    double hier_thresh = 0.7;
     //char *videofile = "./video/result082401_0003.avi";
   //char *videofile = "/home/userver/Qt5.5.1/kcf_multicar/video/result082401_0003.avi";
  //  char *videofile = "/home/nvidia/Video/result133201.avi";//ground
    //char *videofile="rtsp://admin:Admin123@172.16.1.32:554/h264/ch1/main/av_stream";
    //char *videofile = "/home/userver/Videos/0014.avi";
  // char *videofile="rtsp://admin:Admin123@172.16.1.32:554/video2";
  //  char *videofile="rtsp://admin:ak123456@21.192.104.105:554/h264/ch1/sub/av_stream";
      //  char *videofile="rtsp://admin:admin12345@21.192.104.106:554/video1";
    socketinit();
     // char *videofile="/home/liquan/Videos/01.mp4";
    char *videofile="/home/liquan/Videos/01.mp4";

    //char *videofile = "/home/userver/darknet/51.avi";//air
    // Tracker results
    Rect result;
    char data[9];
    int i, j;
    int obj_num = 0;
    AREA obj_rect[50];
    AREA curTag[50], befTag[50];
    int txCent=0,tyCent=0;
    int *xx=(int*)calloc(360*288,sizeof(int));
    int *yy=(int*)calloc(360*288,sizeof(int));
    int pointNum = 0;
    int num_i = 0;


    char track_flag = 0, detect_flag = 0;
    char into_detect_flag = 0, into_track_flag = 0;
    int curNum = 0, befNum = 0;

    // show all boxes
    char box_IP[3];
    char key;
    int key_int;
    char track_IP;
    //VideoCapture cap = cv::VideoCapture(videofile);
    VideoCapture cap;
    Mat Camera_CImg, Process_Img;
    int framenum = 0;

    Mat Process_gray_Img;
    codeBook *cb;
    cb=(codeBook*)calloc(720*576,sizeof(codeBook));
    Mat binary_cb(576,720,CV_8UC1);

    for(i=0;i<720*576;i++)
        binary_cb.data[i]=0;

    //vibe detect
    ViBe_BGS Vibe_Bgs;

    char str[25];
    char *frame_file = "./frame.jpg";
    // the rect at center with size of 720x576
    const Rect detect_rect(1920/2-720/2, 1080/2-576/2, 720, 576);

    cap.open(videofile);
    if (!cap.isOpened()) {
        cout << "Can't open video!" << endl;
        return -1;
    }

//    Detection detection(darknet_path, datacfg, cfgfile, weightfile);

    while (true)
    {
        int wFlag=1, xFlag=0,yFlag=0;
        int tbx=0,tby=0;
        cap >> Camera_CImg;

        if (Camera_CImg.empty()) {
            cout << "Video Ends: 4 0 0" << endl;
            break;
        }
        cv::resize(Camera_CImg,Process_Img,cv::Size(1280,720),0,0,CV_INTER_LINEAR);

//        if(Camera_CImg.cols > 800){//image size:1280*720
//            Process_Img = Camera_CImg(detect_rect);
//        }
//        else{//image size:720*576
//            Process_Img = Camera_CImg;
//        }

        framenum++;
       // cv::resize(Camera_CImg,Camera_CImg,cv::Size(720,576),0,0,CV_INTER_LINEAR);
       // Process_Img = Camera_CImg;
        //cvtColor(Camera_CImg,Process_gray_Img,CV_BGR2GRAY);
        cvtColor(Process_Img,Process_gray_Img,CV_BGR2GRAY);
        if(revdata(data,4)!=-1)
        {
            i=revanalysis(data);
            if(i==1){
            detect_flag = 0;
            track_flag =0;}
            else if(i==0)
            {
                detect_flag = 2;
                track_flag = 2;
              //  fnum=0;
                wFlag=1;
                xFlag=0;
                yFlag=0;
                tbx=0;
                tby=0;
            }
        }

        if(detect_flag == 0 && track_flag == 0){
            initData(curTag, befTag, tagStatus);
            detect_flag = 1;
            into_detect_flag = 1;
            framenum = 1;
        }
        if(detect_flag == 1){
            //obj_num = detection.detect_image(Process_Img, frame_file, thresh, hier_thresh, obj_rect);
            //pointNum = codebook(Process_gray_Img.data,720,576,cb,framenum,binary_cb.data,xx,yy);
            //pointNum = tophatDetect(Process_gray_Img, xx, yy);

            cv::resize(Process_gray_Img, Process_gray_Img,cv::Size(D_WIDTH,D_HEIGHT),0,0,CV_INTER_LINEAR);
            pointNum = vibeDetect(Process_gray_Img, Vibe_Bgs, framenum, xx, yy);
            // printf("pointNum = %d\n", pointNum);

            obj_num = getarea(xx,yy,&pointNum,D_WIDTH,D_HEIGHT,obj_rect);

            befNum = dataAssociation(befTag, befNum, curTag, obj_num, obj_rect, into_detect_flag);
           // printf("befNum = %d\n", befNum);
            into_detect_flag = 0;


            for(i = 0; i < befNum; i++){
               // printf("befTagintNum = %d\n", befTag[i].intNum);
                if(befTag[i].intNum > 20){
                    Mat objpre;
                    Rect objroi;
                    float objresultflag;
                    string labelchar;
                      objroi.x = befTag[i].rect.x *T_WIDTH / D_WIDTH  - 3;
                      objroi.y = befTag[i].rect.y *T_HEIGHT / D_HEIGHT  - 3;
                      objroi.width = befTag[i].rect.w* T_WIDTH / D_WIDTH+ 6;
                      objroi.height = befTag[i].rect.h* T_HEIGHT / D_HEIGHT+ 6;
                      //rectangle(Process_Img, objroi, Scalar( 0, 0, 255 ), 2, 8 );
//                      objpre = Process_gray_Img(objroi);
                      objpre = Process_Img(objroi);

                      objresultflag =  svmppredict(objpre);
                      int  objflag = (int) objresultflag;
                       switch(objflag)
                       {
                       case 0:
                         labelchar = "animals";
                         break;
                       case 1:
                         labelchar = "car";
                         break;
                       case 2:
                         labelchar = "person";
                         break;
                       case 3:
                         labelchar = "Negative";
                         break;
                       }
                        cout<<"预测:"<<labelchar<<endl;

                        cv::String filename = "test";
                        char* ii ;
                        sprintf(ii, "%03X", num_i);
                        filename +=  ii;
                        filename += labelchar;
                        filename += ".jpg";
                        cv::imwrite(filename, objpre);
                         num_i = num_i+1;

//                         imshow(filename,objpre);
//                         waitKey(0);

                    //printf("x = %d, y= %d,width = %d,height = %d\n",objroi.x,objroi.y,objroi.width,objroi.height);
                    //if(objresultflag == 1.0)
//                    if(objflag == 1)
//                    {
//                    //rectangle(Process_Img, Point(befTag[i].rect.x, befTag[i].rect.y ), Point( befTag[i].rect.x+befTag[i].rect.w, befTag[i].rect.y+befTag[i].rect.h), Scalar( 255, 255, 255 ), 1, 8 );
//                    //sprintf(box_IP, "%d", befTag[i].IP - 1);
//                    //putText(Process_Img, box_IP, Point(befTag[i].rect.x, befTag[i].rect.y), FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255), 1, 2, 0);

//                    //result.x = befTag[i].rect.x - 2;
//                    //result.y = befTag[i].rect.y - 2;
//                    //result.width = befTag[i].rect.w + 4;
//                    //result.height = befTag[i].rect.h + 4;
                    result.x = befTag[i].rect.x * T_WIDTH / D_WIDTH  - 2;
                    result.y = befTag[i].rect.y * T_HEIGHT / D_HEIGHT  - 2;
                    result.width = befTag[i].rect.w  * T_WIDTH / D_WIDTH + 4;
                    result.height = befTag[i].rect.h * T_HEIGHT / D_HEIGHT + 4;
                    tune_coordinate(&(result.x), &(result.width), T_WIDTH);
                    tune_coordinate(&(result.y), &(result.height), T_HEIGHT);

                    into_track_flag = 0;
                    track_flag = 1;  //1
                    detect_flag = 0;//1

                    track_IP = i;
                    break;
//                    }
              }

            }
            /*********************************mouse click*************************************
            for(i = 0; i < befNum; i++){
                if(befTag[i].IP > 0){
                    rectangle(Process_Img, Point(befTag[i].rect.x, befTag[i].rect.y ), Point( befTag[i].rect.x+befTag[i].rect.w, befTag[i].rect.y+befTag[i].rect.h), Scalar( 255, 255, 255 ), 1, 8 );
                    sprintf(box_IP, "%d", befTag[i].IP - 1);
                    putText(Process_Img, box_IP, Point(befTag[i].rect.x, befTag[i].rect.y), FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255), 1, 2, 0);
                }
            }


            key = cvWaitKey(10);
            // char to int
            key_int = key - '0';
            if (key == 'q'){
                destroyAllWindows();
                break;
            }
            else if(0 <= key_int && tagStatus[key_int]){
                for(i = 0; i < befNum; i++){
                    if(befTag[i].IP == key_int + 1){
                        result.x = befTag[i].rect.x;
                        result.y = befTag[i].rect.y;
                        result.width = befTag[i].rect.w;
                        result.height = befTag[i].rect.h;

                        into_track_flag = 0;
                        track_flag = 1;
                        detect_flag = 0;

                        track_IP = i;
                        break;

                    }
                    else{
                        continue;
                    }
                }
            }
            else if(key_int != -49){
                cout << "Enter the wrong number!Please enter again!" << endl;
            }
            ***********************************************************************************/

        }

        if(track_flag == 1){
//            cv::resize(Process_Img, Process_gray_Img,cv::Size(T_WIDTH,T_HEIGHT),0,0,CV_INTER_LINEAR);
            track_flag = kcfTrack(Process_Img, result, into_track_flag);
            if(track_flag==0){
                wFlag=0;
                xFlag=0;
                yFlag=0;
                tbx=0;
                tby=0;
            }
            else {
                //flag = 0;
                txCent=result.x + result.width/2;
                tyCent=result.y + result.height/2;
                if(txCent < T_WIDTH/2){
                    xFlag = 0;}
                else {
                    xFlag = 1;
                }
                if(tyCent < T_HEIGHT/2){
                    yFlag = 0;
                }
                else {
                    yFlag = 1;
                }

//                tbx=abs((txCent-T_WIDTH/2)*1920/T_WIDTH);


//                tby=abs((tyCent-T_HEIGHT/2)*1080/T_HEIGHT);
                tbx=abs(txCent-T_WIDTH/2);


                tby=abs(tyCent-T_HEIGHT/2);


            }
//            if(into_track_flag == 1 && track_flag == 1){
//                //sprintf(box_IP, "%d", befTag[track_IP].IP - 1);
//                //putText(Process_Img, box_IP, Point(result.x, result.y), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 0, 255), 1, 2, 0);
//            }
            into_track_flag = 1;
        }

//        // new detection
//        if (track_flag == 0)
//        {
//            cout << "Enter detection: 3 0 0" << endl;
            
//            // clock_t t;
//            // t = clock();

//            //detect_box = detection.detect_image(Process_Img, frame_file, thresh, hier_thresh);//detect maximum probability target
//            //detect_box = detection.detect_image(Process_Img, thresh, hier_thresh);

//            obj_num = detection.detect_image(Process_Img, frame_file, thresh, hier_thresh, obj_rect);


//            // printf("All elapsed time for detecting: %f seconds.\n", sec(clock()-t));

            
//            // TODO create templete
//            cout << "x:" << obj_rect[0].x <<
//                    " y:" << obj_rect[0].y <<
//                    " w:" << obj_rect[0].w <<
//                    " h:" << obj_rect[0].h << endl;

//            if(obj_num == 0){
//                cout << "Detection failed: 2 0 0" << endl;
//            }
//            else{
//                result.x = obj_rect[0].x;
//                result.y = obj_rect[0].y;
//                result.width = obj_rect[0].w;
//                result.height = obj_rect[0].h;

//                //result = tagEdgeExtraction(Process_Img, result);

//                track_flag = kcfTrack(Process_Img, result, track_flag);
//            }
//        }
        
//        // just track
//        else
//        {
//             track_flag = kcfTrack(Process_Img, result, track_flag);
//        }
      //  printf("framenum= %d,wFlag= %d,xFlag= %d,yFlag= %d,tbx= %d,tby= %d\n",framenum,wFlag,xFlag,yFlag,tbx,tby);
        sendanalysis(data,wFlag,xFlag,yFlag,tbx,tby);
        senddata(data,9);
        sprintf(str, "%d", framenum);
        putText(Process_Img, str, Point(100,100), FONT_HERSHEY_PLAIN,
                2.0, Scalar(255, 255, 255), 1, 8, 0);
        imshow("result", Process_Img);

       // imshow("binary_img", binary_cb);
      //  printf("framenum = %d, track_flag = %d\n", framenum, track_flag);
        if (cvWaitKey(1) == 'q')
            break;
    }

    return 0;
}
