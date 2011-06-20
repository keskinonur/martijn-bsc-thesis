/*
 * @video_stage.c
 * @author marc-olivier.dzeukou@parrot.com
 * @date 2007/07/27
 *
 * ihm vision thread implementation
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Stages/vp_stages_i_camif.h>

#include <config.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>
#include <VLIB/Stages/vlib_stage_decode.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <math.h>
#include <sys/time.h> 

#ifndef RECORD_VIDEO
#define RECORD_VIDEO
#endif
#ifdef RECORD_VIDEO
#    include <ardrone_tool/Video/video_stage_recorder.h>
#endif

#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_api.h>

#include "Video/video_stage.h"

IplImage* frontImgStream;

IplImage* OpticFrame;
IplImage* CrtFrame;
IplImage* LastFrame;

IplImage* eig_image;
IplImage* tmp_image;
IplImage* frame_optic;

int seconded;

int win_size = 15;
int corner_count = 500;

CvPoint2D32f cornersA[ 500 ];
CvPoint2D32f cornersB[ 500 ];

CvVideoWriter *writer;
char* file;
int writerbool;
int writing;

#define NB_STAGES 10

PIPELINE_HANDLE pipeline_handle;

IplImage *rgbHeader = NULL;

static uint8_t*  pixbuf_data       = NULL;
static vp_os_mutex_t  video_update_lock = PTHREAD_MUTEX_INITIALIZER;

void recordMode()
{
  // writerbool = !writerbool;
  if(writerbool==1) writerbool=0;
  else if(writerbool==0) writerbool = 1;
}

C_RESULT output_gtk_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  rgbHeader = cvCreateImageHeader(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 3);  //[for colour]
  rgbHeader->imageData = NULL;       //[for colour]

  cvStartWindowThread();
#define frontWindow "DroneView"
  cvNamedWindow(frontWindow, 0);
  cvNamedWindow( "LKpyr_OpticalFlow", 0 );

  frontImgStream = cvCreateImage(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 3);
  OpticFrame = cvCreateImage(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 3);

  CrtFrame = cvCreateImage(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 1);
  LastFrame = cvCreateImage(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 1);

  eig_image = cvCreateImage(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 1);
  tmp_image = cvCreateImage(cvSize(QVGA_WIDTH, QVGA_HEIGHT), IPL_DEPTH_8U, 1);

  seconded = 0;

  // Create Writer
  CvSize size = cvSize(QVGA_WIDTH, QVGA_HEIGHT);
  file = "vid.avi";
  writer = cvCreateVideoWriter(file,CV_FOURCC('M','J','P','G'),10,size,1);
  writerbool = 0;
  writing = 0;

  return (SUCCESS);
}

C_RESULT output_gtk_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&video_update_lock);

  /* Get a reference to the last decoded picture */
  pixbuf_data      = (uint8_t*)in->buffers[0];
  rgbHeader->imageData = (char*)pixbuf_data;  //[for colour]

  cvCvtColor(rgbHeader, frontImgStream, CV_RGB2BGR); //[for colour]
  cvShowImage(frontWindow, frontImgStream);
  cvConvertImage(frontImgStream, CrtFrame, CV_BGR2GRAY);

  // Optical Flow using Lucas Kanade
  if(seconded==1)
  {
    cvGoodFeaturesToTrack( LastFrame, eig_image, tmp_image, cornersA, &corner_count, 0.05, 5.0, 0, 3, 0, 0.04 );
    cvFindCornerSubPix( LastFrame, cornersA, corner_count, cvSize( win_size, win_size ),
        cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );
    // Call Lucas Kanade algorithm
    char features_found[ 500 ];
    float feature_errors[ 500 ];

    CvSize pyr_sz = cvSize(LastFrame->width+8, CrtFrame->height/3 );

    IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_8U, 1 );
    IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_8U, 1 );

    cvCalcOpticalFlowPyrLK( LastFrame, CrtFrame, pyrA, pyrB, cornersA, cornersB, corner_count, 
        cvSize( win_size, win_size ), 5, features_found, feature_errors,
        cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

    // Make an image of the results

    cvConvertImage(frontImgStream, OpticFrame, 0);

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

    int dis;

    CvPoint3D32f tdpoints[550];
    int fp = 0;

    CvMat* points1 = cvCreateMat(1, 550, CV_32FC2);
    CvMat* points2 = cvCreateMat(1, 550, CV_32FC2);

    int crt_p = 0;

    int i;
    for( i=0; i<550; i++ )
    {
      //    printf("Error is %f\n", feature_errors[i]);
      if(feature_errors[i]<0.0001) continue;

      //    printf("Got it\n");
      CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
      CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );

      if(p0.x<1 || p0.x>QVGA_WIDTH) continue;
      if(p0.y<1 || p0.y>QVGA_HEIGHT ) continue;
      if(p1.x<1 || p1.x>QVGA_WIDTH) continue;
      if(p1.y<1 || p1.y>QVGA_HEIGHT ) continue;

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

      int wrong = 0;
      if(x<1 || x>QVGA_WIDTH) wrong = 1;
      if(y<1 || y>QVGA_HEIGHT) wrong = 1;
      int color = 0;
      if(wrong==1) color = 255;

      cvLine(OpticFrame, p0, p1, cvScalar(color,255-color,0,0), 1, 8, 0 );
      cvCircle(OpticFrame, p1, 3, cvScalar(255,0,0,0), 1, 8, 0);   
      // cvPutText(frame_optic, buffer, p0, &font, cvScalar(255, 255, 255, 0));
    }




    // Stereo Vision

    // Record
    // Needs switch to turn on and off again
    if(writerbool==1) writing = 1;

    if(writing==1)  cvWriteFrame(writer, OpticFrame); 

    if(writing==1 && writerbool==0)
    {
      writing = 0;
      cvReleaseVideoWriter(&writer);
    }
  }
  else{
    seconded = 1;
  }

  cvShowImage( "LKpyr_OpticalFlow", OpticFrame );

  cvConvertImage(CrtFrame, LastFrame, 0);

  vp_os_mutex_unlock(&video_update_lock);

  return (SUCCESS);
}



C_RESULT output_gtk_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  cvDestroyWindow(frontWindow);
  cvReleaseImage(&frontImgStream);
  return (SUCCESS);
}


const vp_api_stage_funcs_t vp_stages_output_gtk_funcs =
{
  NULL,
  (vp_api_stage_open_t)output_gtk_stage_open,
  (vp_api_stage_transform_t)output_gtk_stage_transform,
  (vp_api_stage_close_t)output_gtk_stage_close
};

DEFINE_THREAD_ROUTINE(video_stage, data)
{
  C_RESULT res;

  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_api_picture_t picture;

  video_com_config_t              icc;
  vlib_stage_decoding_config_t    vec;
  vp_stages_yuv2rgb_config_t      yuv2rgbconf;
#ifdef RECORD_VIDEO
  video_stage_recorder_config_t   vrc;
#endif
  /// Picture configuration
  picture.format        = PIX_FMT_YUV420P;

  picture.width         = QVGA_WIDTH;
  picture.height        = QVGA_HEIGHT;
  picture.framerate     = 30;

  picture.y_buf   = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT     );
  picture.cr_buf  = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT / 4 );
  picture.cb_buf  = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT / 4 );

  picture.y_line_size   = QVGA_WIDTH;
  picture.cb_line_size  = QVGA_WIDTH / 2;
  picture.cr_line_size  = QVGA_WIDTH / 2;

  vp_os_memset(&icc,          0, sizeof( icc ));
  vp_os_memset(&vec,          0, sizeof( vec ));
  vp_os_memset(&yuv2rgbconf,  0, sizeof( yuv2rgbconf ));

  icc.com                 = COM_VIDEO();
  icc.buffer_size         = 100000;
  icc.protocol            = VP_COM_UDP;
  COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);

  vec.width               = QVGA_WIDTH;
  vec.height              = QVGA_HEIGHT;
  vec.picture             = &picture;
  vec.block_mode_enable   = TRUE;
  vec.luma_only           = FALSE;

  yuv2rgbconf.rgb_format = VP_STAGES_RGB_FORMAT_RGB24;
#ifdef RECORD_VIDEO
  vrc.fp = NULL;
#endif

  pipeline.nb_stages = 0;

  stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
  stages[pipeline.nb_stages].cfg     = (void *)&icc;
  stages[pipeline.nb_stages].funcs   = video_com_funcs;

  pipeline.nb_stages++;

#ifdef RECORD_VIDEO
  stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vrc;
  stages[pipeline.nb_stages].funcs   = video_recorder_funcs;

  pipeline.nb_stages++;
#endif // RECORD_VIDEO
  stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vec;
  stages[pipeline.nb_stages].funcs   = vlib_decoding_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_FILTER_YUV2RGB;
  stages[pipeline.nb_stages].cfg     = (void*)&yuv2rgbconf;
  stages[pipeline.nb_stages].funcs   = vp_stages_yuv2rgb_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SDL;
  stages[pipeline.nb_stages].cfg     = NULL;
  stages[pipeline.nb_stages].funcs   = vp_stages_output_gtk_funcs;

  pipeline.nb_stages++;

  pipeline.stages = &stages[0];

  /* Processing of a pipeline */
  if( !ardrone_tool_exit() )
  {
    PRINT("Video stage thread initialisation\n\n");

    res = vp_api_open(&pipeline, &pipeline_handle);

    if( SUCCEED(res) )
    {
      int loop = SUCCESS;
      out.status = VP_API_STATUS_PROCESSING;


      while( !ardrone_tool_exit() && (loop == SUCCESS) )
      {
        if( SUCCEED(vp_api_run(&pipeline, &out)) ) {
          if( (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING) ) {

            loop = SUCCESS;
          }
        }
        else loop = -1; // Finish this thread
      }


      vp_api_close(&pipeline, &pipeline_handle);
    }
  }

  PRINT("   Video stage thread ended\n\n");

  return (THREAD_RET)0;
}

