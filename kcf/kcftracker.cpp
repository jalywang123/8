///*

//Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure with Kernels (CSK) [2].
//CSK is implemented by using raw gray level features, since it is a single-channel filter.
//KCF is implemented by using HOG features (the default), since it extends CSK to multiple channels.

//[1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
//"High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

//[2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
//"Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.

//Authors: Joao Faro, Christian Bailer, Joao F. Henriques
//Contacts: joaopfaro@gmail.com, Christian.Bailer@dfki.de, henriques@isr.uc.pt
//Institute of Systems and Robotics - University of Coimbra / Department Augmented Vision DFKI


//Constructor parameters, all boolean:
//    hog: use HOG features (default), otherwise use raw pixels
//    fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
//    multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)

//Default values are set for all properties of the tracker depending on the above choices.
//Their values can be customized further before calling init():
//    interp_factor: linear interpolation factor for adaptation
//    sigma: gaussian kernel bandwidth
//    lambda: regularization
//    cell_size: HOG cell size
//    padding: area surrounding the target, relative to its size
//    output_sigma_factor: bandwidth of gaussian target
//    template_size: template size in pixels, 0 to use ROI size
//    scale_step: scale step for multi-scale estimation, 1 to disable it
//    scale_weight: to downweight detection scores of other scales for added stability

//For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

//Inputs to init():
//   image is the initial frame.
//   roi is a cv::Rect with the target positions in the initial frame

//Inputs to update():
//   image is the current frame.

//Outputs of update():
//   cv::Rect with target positions for the current frame


//By downloading, copying, installing or using the software you agree to this license.
//If you do not agree to this license, do not download, install,
//copy or use the software.


//                          License Agreement
//               For Open Source Computer Vision Library
//                       (3-clause BSD License)

//Redistribution and use in source and binary forms, with or without modification,
//are permitted provided that the following conditions are met:

//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.

//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

//  * Neither the names of the copyright holders nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.

//This software is provided by the copyright holders and contributors "as is" and
//any express or implied warranties, including, but not limited to, the implied
//warranties of merchantability and fitness for a particular purpose are disclaimed.
//In no event shall copyright holders or contributors be liable for any direct,
//indirect, incidental, special, exemplary, or consequential damages
//(including, but not limited to, procurement of substitute goods or services;
//loss of use, data, or profits; or business interruption) however caused
//and on any theory of liability, whether in contract, strict liability,
//or tort (including negligence or otherwise) arising in any way out of
//the use of this software, even if advised of the possibility of such damage.
// */

//#ifndef _KCFTRACKER_HEADERS
//#include "kcftracker.hpp"
//#include "ffttools.hpp"
//#include "recttools.hpp"
//#include "fhog.hpp"
//#include "labdata.hpp"
//#endif

//// Constructor
//KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab)
//{

//    // Parameters equal in all cases
//    lambda = 0.0001;
//    padding = 2.5;
//    //output_sigma_factor = 0.1;
//    output_sigma_factor = 0.125;


//    if (hog) {    // HOG
//        // VOT
//        interp_factor = 0.012;
//        sigma = 0.6;
//        // TPAMI
//        //interp_factor = 0.02;
//        //sigma = 0.5;
//        cell_size = 4;
//        _hogfeatures = true;

//        if (lab) {
//            interp_factor = 0.005;
//            sigma = 0.4;
//            //output_sigma_factor = 0.025;
//            output_sigma_factor = 0.1;

//            _labfeatures = true;
//            _labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
//            cell_sizeQ = cell_size*cell_size;
//        }
//        else{
//            _labfeatures = false;
//        }
//    }
//    else {   // RAW
//        interp_factor = 0.075;
//        sigma = 0.2;
//        cell_size = 1;
//        _hogfeatures = false;

//        if (lab) {
//            printf("Lab features are only used with HOG features.\n");
//            _labfeatures = false;
//        }
//    }


//    if (multiscale) { // multiscale
//        template_size = 96;
//        //template_size = 100;
//        scale_step = 1.05;
//        scale_weight = 0.95;
//        if (!fixed_window) {
//            //printf("Multiscale does not support non-fixed window.\n");
//            fixed_window = true;
//        }
//    }
//    else if (fixed_window) {  // fit correction without multiscale
//        template_size = 96;
//        //template_size = 100;
//        scale_step = 1;
//    }
//    else {
//        template_size = 1;
//        scale_step = 1;
//    }
//}

//// Initialize tracker
//void KCFTracker::init(const cv::Rect &roi, cv::Mat image)
//{
//    _roi = roi;
//    assert(roi.width >= 0 && roi.height >= 0);
//    _tmpl = getFeatures(image, 1);
//    _prob = createGaussianPeak(size_patch[0], size_patch[1]);
//    _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
//    //_num = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
//    //_den = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));
//    train(_tmpl, 1.0); // train with initial frame
// }
//// Update position based on the new frame
//cv::Rect KCFTracker::update(cv::Mat image)
//{
//    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 1;
//    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 1;
//    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 2;
//    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 2;

//    float cx = _roi.x + _roi.width / 2.0f;
//    float cy = _roi.y + _roi.height / 2.0f;


//    float peak_value;
//    cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value);

//    float new_peak_value;
//    if (scale_step != 1) {
//        // Test at a smaller _scale
//        cv::Point2f new_res = detect(_tmpl, getFeatures(image, 0, 1.0f / scale_step), new_peak_value);

//        if (scale_weight * new_peak_value > peak_value) {
//            res = new_res;
//            peak_value = new_peak_value;
//            _scale /= scale_step;
//            _roi.width /= scale_step;
//            _roi.height /= scale_step;
//        }

//        // Test at a bigger _scale
//        new_res = detect(_tmpl, getFeatures(image, 0, scale_step), new_peak_value);

//        if (scale_weight * new_peak_value > peak_value) {
//            res = new_res;
//            peak_value = new_peak_value;
//            _scale *= scale_step;
//            _roi.width *= scale_step;
//            _roi.height *= scale_step;
//        }
//    }

//    // Adjust by cell size and _scale
//    _roi.x = cx - _roi.width / 2.0f + ((float) res.x * cell_size * _scale);
//    _roi.y = cy - _roi.height / 2.0f + ((float) res.y * cell_size * _scale);

//    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 1;
//    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 1;
//    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 2;
//    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 2;
//    if (_roi.x + _roi.width >= image.cols) _roi.x = _roi.x - 1;
//    if (_roi.y + _roi.height >= image.rows) _roi.y = _roi.y - 1;

//    assert(_roi.width >= 0 && _roi.height >= 0);
//    cv::Mat x = getFeatures(image, 0);
//    train(x, interp_factor);

//    if(new_peak_value < 0.3 && peak_value < 0.3){
//        _roi.x = 0xffff;
//    }

//    return _roi;
//}


//// Detect object in the current frame.
//cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value)
//{
//    using namespace FFTTools;

//    cv::Mat k = gaussianCorrelation(x, z);
//    cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true)));

//    //minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
//    cv::Point2i pi;
//    double pv;
//    cv::minMaxLoc(res, NULL, &pv, NULL, &pi);
//    peak_value = (float) pv;

//    //subpixel peak estimation, coordinates will be non-integer
//    cv::Point2f p((float)pi.x, (float)pi.y);

//    if (pi.x > 0 && pi.x < res.cols-1) {
//        p.x += subPixelPeak(res.at<float>(pi.y, pi.x-1), peak_value, res.at<float>(pi.y, pi.x+1));
//    }

//    if (pi.y > 0 && pi.y < res.rows-1) {
//        p.y += subPixelPeak(res.at<float>(pi.y-1, pi.x), peak_value, res.at<float>(pi.y+1, pi.x));
//    }

//    p.x -= (res.cols) / 2;
//    p.y -= (res.rows) / 2;

//    return p;
//}

//// train tracker with a single image
//void KCFTracker::train(cv::Mat x, float train_interp_factor)
//{
//    using namespace FFTTools;

//    cv::Mat k = gaussianCorrelation(x, x);
//    cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));
    
//    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
//    _alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor) * alphaf;


//    /*cv::Mat kf = fftd(gaussianCorrelation(x, x));
//    cv::Mat num = complexMultiplication(kf, _prob);
//    cv::Mat den = complexMultiplication(kf, kf + lambda);
    
//    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
//    _num = (1 - train_interp_factor) * _num + (train_interp_factor) * num;
//    _den = (1 - train_interp_factor) * _den + (train_interp_factor) * den;

//    _alphaf = complexDivision(_num, _den);*/

//}

//// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
//cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2)
//{
//    using namespace FFTTools;
//    cv::Mat c = cv::Mat( cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0) );
//    // HOG features
//    if (_hogfeatures) {
//        cv::Mat caux;
//        cv::Mat x1aux;
//        cv::Mat x2aux;
//        for (int i = 0; i < size_patch[2]; i++) {
//            x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
//            x1aux = x1aux.reshape(1, size_patch[0]);
//            x2aux = x2.row(i).reshape(1, size_patch[0]);
//            cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true);
//            caux = fftd(caux, true);
//            rearrange(caux);
//            caux.convertTo(caux,CV_32F);
//            c = c + real(caux);
//        }
//    }
//    // Gray features
//    else {
//        cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
//        c = fftd(c, true);
//        rearrange(c);
//        c = real(c);
//    }
//    cv::Mat d;
//    cv::max(( (cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0])- 2. * c) / (size_patch[0]*size_patch[1]*size_patch[2]) , 0, d);

//    cv::Mat k;
//    cv::exp((-d / (sigma * sigma)), k);
//    return k;
//}

//// Create Gaussian Peak. Function called only in the first frame.
//cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex)
//{
//    cv::Mat_<float> res(sizey, sizex);

//    int syh = (sizey) / 2;
//    int sxh = (sizex) / 2;

//    float output_sigma = std::sqrt((float) sizex * sizey) / padding * output_sigma_factor;
//    float mult = -0.5 / (output_sigma * output_sigma);

//    for (int i = 0; i < sizey; i++)
//        for (int j = 0; j < sizex; j++)
//        {
//            int ih = i - syh;
//            int jh = j - sxh;
//            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
//        }
//    return FFTTools::fftd(res);
//}

//// Obtain sub-window from image, with replication-padding and extract features
//cv::Mat KCFTracker::getFeatures(const cv::Mat & image, bool inithann, float scale_adjust)
//{
//    cv::Rect extracted_roi;

//    float cx = _roi.x + _roi.width / 2;
//    float cy = _roi.y + _roi.height / 2;

//    if (inithann) {
//        int padded_w = _roi.width * padding;
//        int padded_h = _roi.height * padding;
        
//        if (template_size > 1) {  // Fit largest dimension to the given template size
//            if (padded_w >= padded_h)  //fit to width
//                _scale = padded_w / (float) template_size;
//            else
//                _scale = padded_h / (float) template_size;

//            _tmpl_sz.width = padded_w / _scale;
//            _tmpl_sz.height = padded_h / _scale;
//        }
//        else {  //No template size given, use ROI size
//            _tmpl_sz.width = padded_w;
//            _tmpl_sz.height = padded_h;
//            _scale = 1;
//            // original code from paper:
//            /*if (sqrt(padded_w * padded_h) >= 100) {   //Normal size
//                _tmpl_sz.width = padded_w;
//                _tmpl_sz.height = padded_h;
//                _scale = 1;
//            }
//            else {   //ROI is too big, track at half size
//                _tmpl_sz.width = padded_w / 2;
//                _tmpl_sz.height = padded_h / 2;
//                _scale = 2;
//            }*/
//        }

//        if (_hogfeatures) {
//            // Round to cell size and also make it even
//            _tmpl_sz.width = ( ( (int)(_tmpl_sz.width / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
//            _tmpl_sz.height = ( ( (int)(_tmpl_sz.height / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
//        }
//        else {  //Make number of pixels even (helps with some logic involving half-dimensions)
//            _tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
//            _tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
//        }
//    }

//    extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
//    extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;

//    // center roi with new size
//    extracted_roi.x = cx - extracted_roi.width / 2;
//    extracted_roi.y = cy - extracted_roi.height / 2;

//    cv::Mat FeaturesMap;
//    cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);
    
//    if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height) {
//        cv::resize(z, z, _tmpl_sz);
//    }

//    // HOG features
//    if (_hogfeatures) {
//        IplImage z_ipl = z;
//        CvLSVMFeatureMapCaskade *map;
//        getFeatureMaps(&z_ipl, cell_size, &map);
//        normalizeAndTruncate(map,0.2f);
//        PCAFeatureMaps(map);
//        size_patch[0] = map->sizeY;
//        size_patch[1] = map->sizeX;
//        size_patch[2] = map->numFeatures;

//        FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
//        FeaturesMap = FeaturesMap.t();
//        freeFeatureMapObject(&map);

//        // Lab features
//        if (_labfeatures) {
//            cv::Mat imgLab;
//            cvtColor(z, imgLab, CV_BGR2Lab);
//            unsigned char *input = (unsigned char*)(imgLab.data);

//            // Sparse output vector
//            cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch[0]*size_patch[1], CV_32F, float(0));

//            int cntCell = 0;
//            // Iterate through each cell
//            for (int cY = cell_size; cY < z.rows-cell_size; cY+=cell_size){
//                for (int cX = cell_size; cX < z.cols-cell_size; cX+=cell_size){
//                    // Iterate through each pixel of cell (cX,cY)
//                    for(int y = cY; y < cY+cell_size; ++y){
//                        for(int x = cX; x < cX+cell_size; ++x){
//                            // Lab components for each pixel
//                            float l = (float)input[(z.cols * y + x) * 3];
//                            float a = (float)input[(z.cols * y + x) * 3 + 1];
//                            float b = (float)input[(z.cols * y + x) * 3 + 2];

//                            // Iterate trough each centroid
//                            float minDist = FLT_MAX;
//                            int minIdx = 0;
//                            float *inputCentroid = (float*)(_labCentroids.data);
//                            for(int k = 0; k < _labCentroids.rows; ++k){
//                                float dist = ( (l - inputCentroid[3*k]) * (l - inputCentroid[3*k]) )
//                                           + ( (a - inputCentroid[3*k+1]) * (a - inputCentroid[3*k+1]) )
//                                           + ( (b - inputCentroid[3*k+2]) * (b - inputCentroid[3*k+2]) );
//                                if(dist < minDist){
//                                    minDist = dist;
//                                    minIdx = k;
//                                }
//                            }
//                            // Store result at output
//                            outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
//                            //((float*) outputLab.data)[minIdx * (size_patch[0]*size_patch[1]) + cntCell] += 1.0 / cell_sizeQ;
//                        }
//                    }
//                    cntCell++;
//                }
//            }
//            // Update size_patch[2] and add features to FeaturesMap
//            size_patch[2] += _labCentroids.rows;
//            FeaturesMap.push_back(outputLab);
//        }
//    }
//    else {
//        FeaturesMap = RectTools::getGrayImage(z);
//        FeaturesMap -= (float) 0.5; // In Paper;
//        size_patch[0] = z.rows;
//        size_patch[1] = z.cols;
//        size_patch[2] = 1;
//    }
    
//    if (inithann) {
//        createHanningMats();
//    }
//    FeaturesMap = hann.mul(FeaturesMap);
//    return FeaturesMap;
//}
    
//// Initialize Hanning window. Function called only in the first frame.
//void KCFTracker::createHanningMats()
//{
//    cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1],1), CV_32F, cv::Scalar(0));
//    cv::Mat hann2t = cv::Mat(cv::Size(1,size_patch[0]), CV_32F, cv::Scalar(0));

//    for (int i = 0; i < hann1t.cols; i++)
//        hann1t.at<float > (0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
//    for (int i = 0; i < hann2t.rows; i++)
//        hann2t.at<float > (i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

//    cv::Mat hann2d = hann2t * hann1t;
//    // HOG features
//    if (_hogfeatures) {
//        cv::Mat hann1d = hann2d.reshape(1,1); // Procedure do deal with cv::Mat multichannel bug
        
//        hann = cv::Mat(cv::Size(size_patch[0]*size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
//        for (int i = 0; i < size_patch[2]; i++) {
//            for (int j = 0; j<size_patch[0]*size_patch[1]; j++) {
//                hann.at<float>(i,j) = hann1d.at<float>(0,j);
//            }
//        }
//    }
//    // Gray features
//    else {
//        hann = hann2d;
//    }
//}

//// Calculate sub-pixel peak for one dimension
//float KCFTracker::subPixelPeak(float left, float center, float right)
//{
//    float divisor = 2 * center - right - left;

//    if (divisor == 0)
//        return 0;
    
//    return 0.5 * (right - left) / divisor;
//}

//// Calculate the response peak values for image 1 and image 2
//float KCFTracker::tagAreaPeak(cv::Mat image1, cv::Mat image2, float &peak_value)
//{
//    //normalized image
//    int width = (size_patch[1] + 2) * 4;
//    int height = (size_patch[0] + 2) * 4;
//    cv::resize(image1,image1,cv::Size(width,height),0,0,CV_INTER_LINEAR);
//    cv::resize(image2,image2,cv::Size(width,height),0,0,CV_INTER_LINEAR);

//    //Calculate image1 and image2 feature map
//    IplImage z_ipl = image1;
//    CvLSVMFeatureMapCaskade *map;
//    getFeatureMaps(&z_ipl, cell_size, &map);
//    normalizeAndTruncate(map,0.2f);
//    PCAFeatureMaps(map);

//    z_ipl = image2;
//    CvLSVMFeatureMapCaskade *map1;
//    getFeatureMaps(&z_ipl, cell_size, &map1);
//    normalizeAndTruncate(map1,0.2f);
//    PCAFeatureMaps(map1);

//    cv::Mat FeaturesMap;
//    FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
//    FeaturesMap = FeaturesMap.t();
//    freeFeatureMapObject(&map);

//    cv::Mat FeaturesMap1;
//    FeaturesMap1 = cv::Mat(cv::Size(map1->numFeatures,map1->sizeX*map1->sizeY), CV_32F, map1->map);  // Procedure do deal with cv::Mat multichannel bug
//    FeaturesMap1 = FeaturesMap1.t();
//    freeFeatureMapObject(&map1);

//    FeaturesMap = hann.mul(FeaturesMap);
//    FeaturesMap1 = hann.mul(FeaturesMap1);

//    //Calculate response map
//    //max response value-----peak
//    //float peak_value;
//    //cv::Point2f res = detect(FeaturesMap, FeaturesMap1, peak_value);

//    //max response value, min response value, APCE
//    using namespace FFTTools;
//    cv::Mat k = gaussianCorrelation(FeaturesMap, FeaturesMap1);
//    cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true)));

//    //minMaxLoc only accepts doubles for the peak, and integer points for the coordinates
//    double min_pv, max_pv;
//    cv::Point2i max_pi;
//    cv::minMaxLoc(res, &min_pv, &max_pv, NULL, &max_pi);

//    peak_value = (float)max_pv;

//    //subpixel peak estimation, coordinates will be non-integer
//    int pi_x, pi_y;
//    float sum_res = 0;
//    float APCE;

//    for(pi_y = 0; pi_y < res.rows; pi_y++){
//       for(pi_x = 0; pi_x < res.cols; pi_x++){
//           sum_res += (res.at<float>(pi_y, pi_x) - (float)min_pv) * (res.at<float>(pi_y, pi_x) - (float)min_pv);
//       }
//    }

//    APCE = ((float)max_pv - (float)min_pv) * ((float)max_pv - (float)min_pv) * res.rows * res.cols;
//    APCE = APCE / sum_res;

//    return APCE;
//}

/*

Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure with Kernels (CSK) [2].
CSK is implemented by using raw gray level features, since it is a single-channel filter.
KCF is implemented by using HOG features (the default), since it extends CSK to multiple channels.

[1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

[2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.

Authors: Joao Faro, Christian Bailer, Joao F. Henriques
Contacts: joaopfaro@gmail.com, Christian.Bailer@dfki.de, henriques@isr.uc.pt
Institute of Systems and Robotics - University of Coimbra / Department Augmented Vision DFKI


Constructor parameters, all boolean:
    hog: use HOG features (default), otherwise use raw pixels
    fixed_window: fix window size (default), otherwise use ROI size (slower but more accurate)
    multiscale: use multi-scale tracking (default; cannot be used with fixed_window = true)

Default values are set for all properties of the tracker depending on the above choices.
Their values can be customized further before calling init():
    interp_factor: linear interpolation factor for adaptation
    sigma: gaussian kernel bandwidth
    lambda: regularization
    cell_size: HOG cell size
    padding: area surrounding the target, relative to its size
    output_sigma_factor: bandwidth of gaussian target
    template_size: template size in pixels, 0 to use ROI size
    scale_step: scale step for multi-scale estimation, 1 to disable it
    scale_weight: to downweight detection scores of other scales for added stability

For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

Inputs to init():
   image is the initial frame.
   roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
   image is the current frame.

Outputs of update():
   cv::Rect with target positions for the current frame


By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
 */

#ifndef _KCFTRACKER_HEADERS
#include "kcftracker.hpp"
#include "ffttools.hpp"
#include "recttools.hpp"
#include "fhog.hpp"
#include "labdata.hpp"
#endif


// Constructor
KCFTracker::KCFTracker(bool hog, bool fixed_window, bool multiscale, bool lab)
{

    // Parameters equal in all cases
    lambda = 0.0001;
    padding = 2.5;
    //output_sigma_factor = 0.1;
    output_sigma_factor = 0.125;


    if (hog) {    // HOG
        // VOT
        interp_factor = 0.005;// orginal: interp_factor = 0.012. I change it to 0.005
                              // model update speed(weight) in each frame.
        sigma = 0.6;
        // TPAMI
        //interp_factor = 0.02;
        //sigma = 0.5;
        cell_size = 4;
        _hogfeatures = true;

        if (lab) {
            interp_factor = 0.005;
            sigma = 0.4;
            //output_sigma_factor = 0.025;
            output_sigma_factor = 0.1;

            _labfeatures = true;
            _labCentroids = cv::Mat(nClusters, 3, CV_32FC1, &data);
            cell_sizeQ = cell_size*cell_size;
        }
        else{
            _labfeatures = false;
        }
    }
    else {   // RAW
        interp_factor = 0.075;
        sigma = 0.2;
        cell_size = 1;
        _hogfeatures = false;

        if (lab) {
            printf("Lab features are only used with HOG features.\n");
            _labfeatures = false;
        }
    }


    if (multiscale) { // multiscale
        template_size = 96;
        //template_size = 100;
        scale_step = 1.05;
        scale_weight = 0.95;
        if (!fixed_window) {
            //printf("Multiscale does not support non-fixed window.\n");
            fixed_window = true;
        }
    }
    else if (fixed_window) {  // fit correction without multiscale
        template_size = 96;
        //template_size = 100;
        scale_step = 1;
    }
    else {
        template_size = 1;
        scale_step = 1;
    }
}



// Initialize tracker :
// 1. extract roi region's Hog feature x;
// 2. Create training samples' labels y;
// 3. train the regression model
void KCFTracker::init(const cv::Rect &roi, cv::Mat image)
{
    // new add parameters
    mean_APCE = 0;
    mean_Peakmax = 0;
    numCorrect = 0;
    numflag = 0;
    flag = 0;

    // make sure roi region size and
    _roi = roi;
    assert(roi.width >= 0 && roi.height >= 0);

    //extract roi region's Hog feature
    _tmpl = getFeatures(image, 1);

    // Create labels y:  Gaussian Distribution
    _prob = createGaussianPeak(size_patch[0], size_patch[1]);

    // Initialize the parameters of the regression model: a
    _alphaf = cv::Mat(size_patch[0], size_patch[1], CV_32FC2, float(0));

    // train the regression model, namely solve parameters a.
    train(_tmpl, 1.0); // train with initial frame formula (17) in KCF paper
 }


// Update position based on the new frame
cv::Rect KCFTracker::update(cv::Mat image)
{
    // new add
    float APCE,APCE1,APCE2,APCE3;  // APCE values of three scales
    float Belta1 = 0.7;
    float Belta2 = 0.45;

    // Track Search Window :
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 1;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 1;
    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 2;
    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 2;
    float cx = _roi.x + _roi.width / 2.0f;  // track window's center,namely target center given.
    float cy = _roi.y + _roi.height / 2.0f;

    // Compute Response Map and Return the maxest Peakvalue point,namely res
    float peak_value;
    cv::Point2f res = detect(_tmpl, getFeatures(image, 0, 1.0f), peak_value, APCE1);
    APCE = APCE1;

    // Multi-scales : Find the maxest Peakvalue point of anthor two scales
    if (scale_step != 1) {
        // Test at a smaller _scale
        float new_peak_value;
        cv::Point2f new_res = detect(_tmpl, getFeatures(image, 0, 1.0f / scale_step), new_peak_value, APCE2);

        // By comparison,determine which scale's resulte best
        if (scale_weight * new_peak_value > peak_value) {
            res = new_res;
            peak_value = new_peak_value;
            _scale /= scale_step;
            _roi.width /= scale_step;
            _roi.height /= scale_step;
            APCE = APCE2;
        }

        // Test at a bigger _scale
        new_res = detect(_tmpl, getFeatures(image, 0, scale_step), new_peak_value, APCE3);

        // By comparison,determine which scale's resulte best
        if (scale_weight * new_peak_value > peak_value) {
            res = new_res;
            peak_value = new_peak_value;
            _scale *= scale_step;
            _roi.width *= scale_step;
            _roi.height *= scale_step;
            APCE = APCE3;
        }
    }
   // printf(" peak_max = %f , APCE = %f \n",peak_value,APCE);


    // Adjust by cell size and _scale , get the target location in current image
    _roi.x = cx - _roi.width / 2.0f + ((float) res.x * cell_size * _scale);
    _roi.y = cy - _roi.height / 2.0f + ((float) res.y * cell_size * _scale);
    if (_roi.x >= image.cols - 1) _roi.x = image.cols - 1;
    if (_roi.y >= image.rows - 1) _roi.y = image.rows - 1;
    if (_roi.x + _roi.width <= 0) _roi.x = -_roi.width + 2;
    if (_roi.y + _roi.height <= 0) _roi.y = -_roi.height + 2;


    // ******************************   Track Error Judgement and Model Update Judgement  **********************
    // 1. Model Update Judgement
    if( peak_value >= Belta1 * mean_Peakmax && APCE >= Belta2 * mean_APCE )
    {
        // update mean of APCE and Peakmax
        mean_APCE = ( mean_APCE * numCorrect + APCE ) / ( numCorrect + 1 );  //
        mean_Peakmax = ( mean_Peakmax * numCorrect + peak_value ) / ( numCorrect + 1 );
        numCorrect = numCorrect + 1;
        // printf("numCorrect = %d \n",numCorrect);
        numflag = 0;
        flag = 0;

        // update trained model by target in current frame
       assert(_roi.width >= 0 && _roi.height >= 0);
       cv::Mat x = getFeatures(image, 0); // get the target region roi's Hog feature
       train(x, interp_factor);//update model parameters
    }
    // 2. Track Error Judgement : Occlusion, Disappear, Error
    else if ( peak_value < 0.3 && APCE <= 30 && peak_value < Belta1 * mean_Peakmax && APCE < Belta2 * mean_APCE)
    {
     numflag++;
    }

    // track error,return 1
    if(numflag > 3) // 10 -> 3
        flag = 1;

    // return the target location in current frame;
    return _roi;
}


// Detect object in the current frame
cv::Point2f KCFTracker::detect(cv::Mat z, cv::Mat x, float &peak_value,float &APCE)
{
    using namespace FFTTools;

    // Using Gaussion Kernel Function K, Compute the Cross-correlation between x and z
    // namely K(x,z) = <x,z>
    // x : target region roi in before frame
    // z : candidate region roi in current frame
    cv::Mat k = gaussianCorrelation(x, z);

    // Compute Respose Map :
    // _alphaf : the regression model parameter
    // the Response Map's size is equal to hog feature map's size
    cv::Mat res = (real(fftd(complexMultiplication(_alphaf, fftd(k)), true))); // respose map


    // Find the minest and maxest value in response map
    cv::Point2i pi;
    cv::Point2i pi_min; // the point corresponding to the minest value
    cv::Point2i pi_max; // the point corresponding to the maxest value
    double pv;
    double pv_min;    // the minest value
    double pv_max;    // the maxest value
    cv::minMaxLoc(res, &pv_min, &pv_max, &pi_min, &pi_max); // find min and max response value


    // Compute APCE : Refer to the paper LMCF (Large Margin Object Tracking with Circulant Features Maps,CVPR 2017)
    int pi_x,pi_y;
    float sum_res = 0;

    for(pi_y = 0; pi_y < res.rows; pi_y++)  // sum
    {
        for(pi_x = 0; pi_x < res.cols; pi_x++)
        {
            sum_res += (res.at<float>(pi_y,pi_x) - (float)pv_min) * (res.at<float>(pi_y,pi_x) - (float)pv_min);
        }
    }
    APCE = ((float)pv_max - (float)pv_min) * ((float)pv_max - (float)pv_min) * res.rows * res.cols;
    APCE = APCE / sum_res;
    pv = pv_max;
    pi=pi_max;
    peak_value = (float) pv;

    // Get the true location of target in current image
    //subpixel peak estimation, coordinates will be non-integer
    cv::Point2f p((float)pi.x, (float)pi.y);

    if (pi.x > 0 && pi.x < res.cols-1) {
        p.x += subPixelPeak(res.at<float>(pi.y, pi.x-1), peak_value, res.at<float>(pi.y, pi.x+1));
    }

    if (pi.y > 0 && pi.y < res.rows-1) {
        p.y += subPixelPeak(res.at<float>(pi.y-1, pi.x), peak_value, res.at<float>(pi.y+1, pi.x));
    }

    p.x -= (res.cols) / 2;
    p.y -= (res.rows) / 2;

    return p;
}

// train tracker with a single image
void KCFTracker::train(cv::Mat x, float train_interp_factor)  // x : feature map
{
    using namespace FFTTools;

    // Using Gaussion Kernel Function K, Compute the Auto-correlation of x
    // namely K(x,x) = <x,x>
    // x : target region roi in first frame
    cv::Mat k = gaussianCorrelation(x, x);

    // Train the regression model with the first frame:
    // namely solve the parameter a of model refer to formula (17) in KCF paper.
    cv::Mat alphaf = complexDivision(_prob, (fftd(k) + lambda));
    _tmpl = (1 - train_interp_factor) * _tmpl + (train_interp_factor) * x;
    _alphaf = (1 - train_interp_factor) * _alphaf + (train_interp_factor) * alphaf;

}


// Using A Gaussian Kernel K, Evaluate cross-correlation K(X,Y) or auto-correlation K(X,X)
// Definition of the formula refer to KCF paper
// Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y,
// which must both be MxN. They must  also be periodic (ie., pre-processed with a cosine window).
cv::Mat KCFTracker::gaussianCorrelation(cv::Mat x1, cv::Mat x2)
{
    using namespace FFTTools;
    cv::Mat c = cv::Mat( cv::Size(size_patch[1], size_patch[0]), CV_32F, cv::Scalar(0) );
    // HOG features
    if (_hogfeatures) {
        cv::Mat caux;
        cv::Mat x1aux;
        cv::Mat x2aux;
        for (int i = 0; i < size_patch[2]; i++) {
            x1aux = x1.row(i);   // Procedure do deal with cv::Mat multichannel bug
            x1aux = x1aux.reshape(1, size_patch[0]);
            x2aux = x2.row(i).reshape(1, size_patch[0]);
            cv::mulSpectrums(fftd(x1aux), fftd(x2aux), caux, 0, true);
            caux = fftd(caux, true);
            rearrange(caux);
            caux.convertTo(caux,CV_32F);
            c = c + real(caux);
        }
    }
    // Gray features
    else {
        cv::mulSpectrums(fftd(x1), fftd(x2), c, 0, true);
        c = fftd(c, true);
        rearrange(c);
        c = real(c);
    }
    cv::Mat d;
    cv::max(( (cv::sum(x1.mul(x1))[0] + cv::sum(x2.mul(x2))[0])- 2. * c) / (size_patch[0]*size_patch[1]*size_patch[2]) , 0, d);

    cv::Mat k;
    cv::exp((-d / (sigma * sigma)), k);
    return k;
}

// Create train sample's labels y : gaussion distribution
// Create Gaussian Peak. Function called only in the first frame.
cv::Mat KCFTracker::createGaussianPeak(int sizey, int sizex)
{
    cv::Mat_<float> res(sizey, sizex);

    int syh = (sizey) / 2;
    int sxh = (sizex) / 2;

    float output_sigma = std::sqrt((float) sizex * sizey) / padding * output_sigma_factor;
    float mult = -0.5 / (output_sigma * output_sigma);

    for (int i = 0; i < sizey; i++)
        for (int j = 0; j < sizex; j++)
        {
            int ih = i - syh;
            int jh = j - sxh;
            res(i, j) = std::exp(mult * (float) (ih * ih + jh * jh));
        }
    return FFTTools::fftd(res);
}

// Obtain sub-window from image, with replication-padding and extract features
// First, Get padded roi region by padding target region
// Second, Extract roi region's Hog feature vector
// Third, Create a Hanning window function (a cosine window)
// lastly, Get a periodic Hog feature vector by multiplying the above Hog feature vector with Hanning window function
cv::Mat KCFTracker::getFeatures(const cv::Mat & image, bool inithann, float scale_adjust)
{
    cv::Rect extracted_roi;
    // target Center location
    float cx = _roi.x + _roi.width / 2;
    float cy = _roi.y + _roi.height / 2;

    if (inithann) {
        int padded_w = _roi.width * padding;
        int padded_h = _roi.height * padding;

        if (template_size > 1) {  // Fit largest dimension to the given template size
            if (padded_w >= padded_h)  //fit to width
                _scale = padded_w / (float) template_size;
            else
                _scale = padded_h / (float) template_size;

            _tmpl_sz.width = padded_w / _scale;
            _tmpl_sz.height = padded_h / _scale;
        }
        else {  //No template size given, use ROI size
            _tmpl_sz.width = padded_w;
            _tmpl_sz.height = padded_h;
            _scale = 1;
            // original code from paper:
            /*if (sqrt(padded_w * padded_h) >= 100) {   //Normal size
                _tmpl_sz.width = padded_w;
                _tmpl_sz.height = padded_h;
                _scale = 1;
            }
            else {   //ROI is too big, track at half size
                _tmpl_sz.width = padded_w / 2;
                _tmpl_sz.height = padded_h / 2;
                _scale = 2;
            }*/
        }

        if (_hogfeatures) {
            // Round to cell size and also make it even
            _tmpl_sz.width = ( ( (int)(_tmpl_sz.width / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
            _tmpl_sz.height = ( ( (int)(_tmpl_sz.height / (2 * cell_size)) ) * 2 * cell_size ) + cell_size*2;
        }
        else {  //Make number of pixels even (helps with some logic involving half-dimensions)
            _tmpl_sz.width = (_tmpl_sz.width / 2) * 2;
            _tmpl_sz.height = (_tmpl_sz.height / 2) * 2;
        }
    }

    // roi region's size and top left location : roi center location is same as targets' given.
    extracted_roi.width = scale_adjust * _scale * _tmpl_sz.width;
    extracted_roi.height = scale_adjust * _scale * _tmpl_sz.height;
    extracted_roi.x = cx - extracted_roi.width / 2; // top left position
    extracted_roi.y = cy - extracted_roi.height / 2;

    // Extract roi region from image
    cv::Mat FeaturesMap;
    cv::Mat z = RectTools::subwindow(image, extracted_roi, cv::BORDER_REPLICATE);

    if (z.cols != _tmpl_sz.width || z.rows != _tmpl_sz.height) {
        cv::resize(z, z, _tmpl_sz);
    }

    // HOG features
    //   ori region divides into several small region,each region get a 31 dimensons hog feature

    if (_hogfeatures) {
        IplImage z_ipl = z;
        CvLSVMFeatureMapCaskade *map;
        getFeatureMaps(&z_ipl, cell_size, &map);
        normalizeAndTruncate(map,0.2f);
        PCAFeatureMaps(map);// PCA-HOG
        size_patch[0] = map->sizeY;
        size_patch[1] = map->sizeX;
        size_patch[2] = map->numFeatures; // 31= 3 x Orients + 4; Orients = 9

        FeaturesMap = cv::Mat(cv::Size(map->numFeatures,map->sizeX*map->sizeY), CV_32F, map->map);  // Procedure do deal with cv::Mat multichannel bug
        FeaturesMap = FeaturesMap.t();//matrix transpose : rows data->columns data, columns data -> rows data
        freeFeatureMapObject(&map);

        // Lab features
        if (_labfeatures) {
            cv::Mat imgLab;
            cvtColor(z, imgLab, CV_BGR2Lab);
            unsigned char *input = (unsigned char*)(imgLab.data);

            // Sparse output vector
            cv::Mat outputLab = cv::Mat(_labCentroids.rows, size_patch[0]*size_patch[1], CV_32F, float(0));

            int cntCell = 0;
            // Iterate through each cell
            for (int cY = cell_size; cY < z.rows-cell_size; cY+=cell_size){
                for (int cX = cell_size; cX < z.cols-cell_size; cX+=cell_size){
                    // Iterate through each pixel of cell (cX,cY)
                    for(int y = cY; y < cY+cell_size; ++y){
                        for(int x = cX; x < cX+cell_size; ++x){
                            // Lab components for each pixel
                            float l = (float)input[(z.cols * y + x) * 3];
                            float a = (float)input[(z.cols * y + x) * 3 + 1];
                            float b = (float)input[(z.cols * y + x) * 3 + 2];

                            // Iterate trough each centroid
                            float minDist = FLT_MAX;
                            int minIdx = 0;
                            float *inputCentroid = (float*)(_labCentroids.data);
                            for(int k = 0; k < _labCentroids.rows; ++k){
                                float dist = ( (l - inputCentroid[3*k]) * (l - inputCentroid[3*k]) )
                                           + ( (a - inputCentroid[3*k+1]) * (a - inputCentroid[3*k+1]) )
                                           + ( (b - inputCentroid[3*k+2]) * (b - inputCentroid[3*k+2]) );
                                if(dist < minDist){
                                    minDist = dist;
                                    minIdx = k;
                                }
                            }
                            // Store result at output
                            outputLab.at<float>(minIdx, cntCell) += 1.0 / cell_sizeQ;
                            //((float*) outputLab.data)[minIdx * (size_patch[0]*size_patch[1]) + cntCell] += 1.0 / cell_sizeQ;
                        }
                    }
                    cntCell++;
                }
            }
            // Update size_patch[2] and add features to FeaturesMap
            size_patch[2] += _labCentroids.rows;
            FeaturesMap.push_back(outputLab);
        }
    }
    else {
        FeaturesMap = RectTools::getGrayImage(z);
        FeaturesMap -= (float) 0.5; // In Paper;
        size_patch[0] = z.rows;
        size_patch[1] = z.cols;
        size_patch[2] = 1;
    }

    // Create a Hanning Window function, which is a cosine window.
    // It can turn a hog feature vector into a periodic vector by multiply.
    if (inithann) {
        createHanningMats();
    }

    // get a periodic hog feature vector
    FeaturesMap = hann.mul(FeaturesMap);
    return FeaturesMap;
}

// Initialize Hanning window. Function called only in the first frame.
void KCFTracker::createHanningMats()
{
    cv::Mat hann1t = cv::Mat(cv::Size(size_patch[1],1), CV_32F, cv::Scalar(0));
    cv::Mat hann2t = cv::Mat(cv::Size(1,size_patch[0]), CV_32F, cv::Scalar(0));

    for (int i = 0; i < hann1t.cols; i++)
        hann1t.at<float > (0, i) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann1t.cols - 1)));
    for (int i = 0; i < hann2t.rows; i++)
        hann2t.at<float > (i, 0) = 0.5 * (1 - std::cos(2 * 3.14159265358979323846 * i / (hann2t.rows - 1)));

    cv::Mat hann2d = hann2t * hann1t;
    // HOG features
    if (_hogfeatures) {
        cv::Mat hann1d = hann2d.reshape(1,1); // Procedure do deal with cv::Mat multichannel bug

        hann = cv::Mat(cv::Size(size_patch[0]*size_patch[1], size_patch[2]), CV_32F, cv::Scalar(0));
        for (int i = 0; i < size_patch[2]; i++) {
            for (int j = 0; j<size_patch[0]*size_patch[1]; j++) {
                hann.at<float>(i,j) = hann1d.at<float>(0,j);
            }
        }
    }
    // Gray features
    else {
        hann = hann2d;
    }
}

// Calculate sub-pixel peak for one dimension
float KCFTracker::subPixelPeak(float left, float center, float right)
{
    float divisor = 2 * center - right - left;

    if (divisor == 0)
        return 0;

    return 0.5 * (right - left) / divisor;
}


