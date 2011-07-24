/********************************************************************
 *                    COPYRIGHT PARROT 2010
 ********************************************************************
 *       PARROT - A.R.Drone SDK Windows Client Example
 *-----------------------------------------------------------------*/

/*
Warning !!!!
This file is normally automatically generated by the A.R. Drone SDK Makefile.
This is a local copy manually modified for use with the Windows client demonstration.

Parameters concerning network configuration (IP adress, Wifi SSID ...)
are not used, since we let Windows handle this.

Parameters concerning the video resolution must be set accordingly to the parameters
used to compile the drone software (here are the default which were certainly used
for the public-released drones.
*/

#ifndef _GENERATED_CUSTOM_CONFIGURATION_H_
#define _GENERATED_CUSTOM_CONFIGURATION_H_

#if  defined(BR2_PACKAGE_BCM4318_AP)
#  define AP
#else
#  define STA
#endif


#define DRONE_VIDEO_MAX_WIDTH 640
#define DRONE_VIDEO_MAX_HEIGHT 480

#define TEXTURE_WIDTH 1024
#define TEXTURE_HEIGHT 1024

//#define WINDOW_WIDTH 640
//#define WINDOW_HEIGHT 480

#define WINDOW_WIDTH 200
#define WINDOW_HEIGHT 175

//#define WINDOW_WIDTH 400
//#define WINDOW_HEIGHT 320

#define MAJOR_VERSION 1
#define MINOR_VERSION 3
#define MODIF_VERSION 0
#define CURRENT_NUM_VERSION_SOFT "1.3.0"
#define CURRENT_BUILD_DATE __DATE__

#define CAMIF_V_CAMERA_USED CAMIF_CAMERA_CRESYN
#define CAMIF_H_CAMERA_USED CAMIF_CAMERA_OVTRULY
#define CAMIF_V_RESOLUTION_USED CAMIF_RES_QCIF
#define CAMIF_H_RESOLUTION_USED CAMIF_RES_VGA
#define CAMIF_V_FRAMERATE_USED 60
#define CAMIF_H_FRAMERATE_USED 15
#define V_ACQ_WIDTH  (QCIF_WIDTH)
#define V_ACQ_HEIGHT (QCIF_HEIGHT)
#define H_ACQ_WIDTH  (VGA_WIDTH)
#define H_ACQ_HEIGHT (VGA_HEIGHT)
#define STREAM_WIDTH  (QVGA_WIDTH)
#define STREAM_HEIGHT (QVGA_HEIGHT)
#define STREAM_RESOLUTION_USED CAMIF_RES_QVGA
#define STREAM_FRAMERATE CAMIF_RES_
#define USE_VIDEO_YUV

#define PRINT_WHERE (UART_PRINT|WIFI_PRINT|FLASH_PRINT)

#define ADC_CMD_SELECT_ULTRASOUND ADC_CMD_SELECT_ULTRASOUND_25Hz

#define HELICES_PARROT
#define MECA_NO_SHELL
#define INSIDE_FLIGHT
#define NAVDATA_ALL
#define DRONE_ORIENTATION_45_DEG
#define BRUSHLESS
#define CARD_VERSION 0x10

#define WIFI_NETWORK_NAME "ardrone_RC130"
#define WIFI_BROADCAST "192.168.1.255"
#define WIFI_ARDRONE_IP "192.168.1.1"

#define WIFI_MOBILE_IP "192.168.1.2"

#endif // ! _GENERATED_CUSTOM_CONFIGURATION_H_
