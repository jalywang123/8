#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect/objdetect.hpp>


#include <iostream>
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <sys/time.h>
#include <QDebug>
#include "svm.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;



float svmppredict(Mat orimg)
{
Mat img,testFeatureMat,testImg;
float resultflag;
int i;
Ptr<SVM> svm = SVM::create();  //
//svm = StatModel::load<SVM>("SVM_DATA.xml");
//svm = StatModel::load<SVM>("Classifier.xml");
//svm = StatModel::load<SVM>("/home/liquan/PycharmProjects/image_augmentor-master/samples2/Classifier.xml");
 svm = StatModel::load<SVM>("/home/liquan/QtProjects/svm/svm_train_opencv3/samples_caijian/Classifier_caijian_Negative.xml");
// svm = StatModel::load<SVM>("/home/liquan/PycharmProjects/Image-Classification-Using-SVM-master/src/model_file.xml");
//svm.load("/home/liquan/QtProjects/svm_opencv/car_(pos0402%2Bneg0402)007_64x64.xml");

HOGDescriptor hog(cvSize(32,32), cvSize(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
//HOGDescriptor *hog = new HOGDescriptor(Size(64,64),Size(16,16),Size(8,8),Size(8,8),9);
vector<float> descriptor;

//orimg = imread("/home/liquan/Pictures/car1.jpg");
if(orimg.data == 0)
{
   printf("no test image !!!");
   return 0;
}
if(orimg.channels() == 3)
{
    cvtColor(orimg,img,CV_BGR2GRAY);
}
else
{
    orimg.copyTo(img);
}
// predict

//   double time_Start = (double)clock();
   resize(img,testImg,cvSize(32,32));
   hog.compute(testImg,descriptor);  // compute hog feature
   testFeatureMat = Mat::zeros(1,descriptor.size(),CV_32FC1);
   for(i= 0; i < descriptor.size();i++)   // copy descriptor to testFeatureMat
     {
        testFeatureMat.at<float>(0,i) = descriptor[i];
     }

    // result = svm.predict(testFeatureMat);
     resultflag = svm->predict(testFeatureMat);
//     double time_End = (double)clock();
     cout << "resultflag = " << resultflag <<endl;
    // qDebug()<<(time_End - time_Start)<<"ms";

return resultflag;

}
