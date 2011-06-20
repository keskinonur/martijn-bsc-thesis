#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

#include <sstream>

const int MAX_CORNERS = 1000;

int main(int argc, char* argv[]) {

  // Recorder
  char* filename = argv[1];
  CvCapture* capturedevice = cvCreateFileCapture(filename); 

  // Captured Images
  IplImage* frame_capt = cvQueryFrame(capturedevice);
  CvSize img_sz = cvGetSize(frame_capt);

  IplImage* frame_crt = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
  IplImage* frame_last = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);

  CvMat* disparity = cvCreateMat(img_sz.height, img_sz.width, CV_16S);
  CvMat* vdisparity = cvCreateMat(img_sz.height, img_sz.width, CV_8U);

  cvConvertImage(frame_capt, frame_last, CV_CVTIMG_FLIP);

  // Images for modification
  IplImage* frame_optic = cvCreateImage(img_sz, frame_capt->depth, frame_capt->nChannels);

  IplImage* eig_image = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
  IplImage* tmp_image = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);

  IplImage* override = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);

  // Image Windows
  cvNamedWindow( "ImageA", 0 );
  cvNamedWindow( "ImageB", 0 );
  cvNamedWindow( "LKpyr_OpticalFlow", 0 );

  int win_size = 15;
  int corner_count = MAX_CORNERS;
  CvPoint2D32f* cornersA = new CvPoint2D32f[MAX_CORNERS];

  int k=0;

  while (cvGrabFrame(capturedevice)){
    frame_capt = override; frame_crt = override;
    eig_image = override; tmp_image = override;

    frame_capt = cvRetrieveFrame(capturedevice);

    if(k<10)
    {
      k++;
      continue;

    }else{
      k=0;
    }

    cvConvertImage(frame_capt, frame_crt);

    cvGoodFeaturesToTrack( frame_last, eig_image, tmp_image, cornersA, &corner_count, 0.05, 5.0, 0, 3, 0, 0.04 );
    cvFindCornerSubPix( frame_last, cornersA, corner_count, cvSize( win_size, win_size ),
        cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

    // Call Lucas Kanade algorithm
    char features_found[ MAX_CORNERS ];
    float feature_errors[ MAX_CORNERS ];

    CvSize pyr_sz = cvSize( frame_last->width+8, frame_crt->height/3 );

    IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_8U, 1 );
    IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_8U, 1 );

    CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];

    cvCalcOpticalFlowPyrLK( frame_last, frame_crt, pyrA, pyrB, cornersA, cornersB, corner_count, 
        cvSize( win_size, win_size ), 5, features_found, feature_errors,
        cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

    // Make an image of the results

    cvConvertImage(frame_capt, frame_optic);

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

    int dis;

    CvPoint3D32f* tdpoints = new CvPoint3D32f[550];
    int fp = 0;

    CvMat* points1 = cvCreateMat(1, 550, CV_32FC2);
    CvMat* points2 = cvCreateMat(1, 550, CV_32FC2);

    int crt_p = 0;

    for( int i=0; i<550; i++ ){
      //    printf("Error is %f\n", feature_errors[i]);
      if(feature_errors[i]<0.001) continue;

      //    printf("Got it\n");
      CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
      CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );

      if(p0.x<1 || p0.x>img_sz.width) continue;
      if(p0.y<1 || p0.y>img_sz.height) continue;
      if(p1.x<1 || p1.x>img_sz.width) continue;
      if(p1.y<1 || p1.y>img_sz.height) continue;

      points1->data.fl[crt_p*2] =   p0.x;
      points1->data.fl[crt_p*2+1] = p0.y;

      points2->data.fl[crt_p*2] =   p1.x;
      points2->data.fl[crt_p*2+1] = p1.y;

      crt_p++;


      dis = (int) sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
      //    printf("(%d,%d)(%d,%d): %d\n", p0.x, p0.y, p1.x, p1.y, dis);

      float x = (float) p0.x;
      float y = (float) p0.y;
      float z = (float) dis;
      CvPoint3D32f p = cvPoint3D32f(x, y, z);

      tdpoints[fp] = p;
      fp++;

      char buffer[50];
      sprintf( buffer, "%d", dis);

      bool wrong = false;
      if(x<1 || x>img_sz.width) wrong = true;
      if(y<1 || y>img_sz.height) wrong = true;
      int color = 0;
      if(wrong) color = 255;

      cvLine( frame_optic, p0, p1, CV_RGB(255-color,color,0), .3 );
      cvCircle(frame_optic, p1, 3, cvScalar(255,0,0), 1);   
      // cvPutText(frame_optic, buffer, p0, &font, cvScalar(255, 255, 255, 0));

    }

    cvShowImage( "LKpyr_OpticalFlow", frame_optic );


    // Stereo Vision
    if(!(crt_p>7)) continue;

    printf("Stereo Vision\n");

    CvMat* frame_last_r = cvCreateMat(img_sz.height, img_sz.width, CV_8U);
    CvMat* frame_crt_r = cvCreateMat(img_sz.height, img_sz.width, CV_8U);

    double M1[3][3], M2[3][3], D1[5], D2[5];

    double R1[3][3], R2[3][3];
    CvMat _R1 = cvMat(3,3, CV_64F, R1);
    CvMat _R2 = cvMat(3,3, CV_64F, R2);

    CvMat _M1 = cvMat(3,3, CV_64F, M1);
    CvMat _M2 = cvMat(3,3, CV_64F, M2);
    CvMat _D1 = cvMat(1,5, CV_64F, D1);
    CvMat _D2 = cvMat(1,5, CV_64F, D2);

    CvMat* mx1 = cvCreateMat(img_sz.height, img_sz.width, CV_32F);
    CvMat* my1 = cvCreateMat(img_sz.height, img_sz.width, CV_32F);
    CvMat* mx2 = cvCreateMat(img_sz.height, img_sz.width, CV_32F);
    CvMat* my2 = cvCreateMat(img_sz.height, img_sz.width, CV_32F);

    double H1[3][3], H2[3][3], iM[3][3];
    CvMat _H1 = cvMat(3,3, CV_64F, H1);
    CvMat _H2 = cvMat(3,3, CV_64F, H2);
    CvMat _iM = cvMat(3,3, CV_64F, iM);

    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);

    CvMat* fundamental_matrix = cvCreateMat(3,3, CV_32FC1);
    int fm_count = cvFindFundamentalMat(points1, points2, fundamental_matrix, CV_FM_RANSAC, 1.0, 0.99, NULL);

    printf("Fundamental Matrix: %d\n", fm_count);
    if(fm_count==0) continue;

    //    CvMat* correspondent_lines = cvCreateMat(3, 550, CV_32FC1);
    //    cvComputeCorrespondEpilines(points1, 1, fundamental_matrix, correspondent_lines);

    cvStereoRectifyUncalibrated(points1, points2, fundamental_matrix, img_sz, &_H1, &_H2,3);

    cvInvert(&_M1, &_iM);
    cvMatMul(&_H1, &_M1, &_R1);
    cvMatMul(&_iM, &_R1, &_R1);  

    cvInvert(&_M2, &_iM);
    cvMatMul(&_H2, &_M2, &_R2);
    cvMatMul(&_iM, &_R2, &_R2);

    cvInitUndistortRectifyMap(&_M1, &_D1, &_R1, &_M1, mx1, my1);
    cvInitUndistortRectifyMap(&_M2, &_D1, &_R2, &_M2, mx2, my2);

    cvRemap(frame_last, frame_last_r, mx1, my1);
    cvRemap(frame_crt, frame_crt_r, mx2, my2);

    CvStereoBMState* BMState = cvCreateStereoBMState(CV_STEREO_BM_NARROW);
    cvFindStereoCorrespondenceBM(frame_last_r, frame_crt_r, disparity, BMState);

    cvNormalize(disparity, vdisparity, 0 , 256, CV_MINMAX);

    cvNamedWindow("Disparity", 0);
    cvShowImage("Disparity", vdisparity);

    cvShowImage( "ImageA", frame_last_r);
    cvShowImage( "ImageB", frame_crt_r);

    cvWaitKey(0);

    cvConvertImage(frame_crt, frame_last, 0);
  }

  return 0;
}
