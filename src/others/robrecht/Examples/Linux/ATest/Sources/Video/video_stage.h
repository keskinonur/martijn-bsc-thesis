#ifndef _IHM_STAGES_O_GTK_H
#define _IHM_STAGES_O_GTK_H

#include <config.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <opencv/highgui.h>

PROTO_THREAD_ROUTINE(video_stage, data);
void recordMode();

#endif // _IHM_STAGES_O_GTK_H
