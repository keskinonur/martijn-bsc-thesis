#pragma once

#include "slam_queue.h"
#include "slam_module_frame.h"
#include "slam_module_sensor.h"
#include "slam_module_ui.h"

#include "opencv2/core/types_c.h"

struct bot_ardrone_frame;
struct bot_ardrone_measurement;
struct bot_ardrone_control;

// threads
static DWORD WINAPI start_process_frame(void* Param);
static DWORD WINAPI start_process_sensor(void* Param);
static DWORD WINAPI start_ui(void* Param);


class slam
{
public:
	slam();
	~slam();
	void slam::run();

	void add_input_frame(bot_ardrone_frame *f);
	void add_input_sensor(bot_ardrone_measurement *m);


	/* threads */
	HANDLE thread_process_frame;
	HANDLE thread_process_sensor;
	HANDLE thread_ui;

	/* queues */
	slam_queue<bot_ardrone_frame*> queue_frame;
	slam_queue<bot_ardrone_measurement*> queue_sensor;
	slam_queue<bot_ardrone_control*> queue_control;

	/* modules (processors) */
	slam_module_frame *m_frame;
	slam_module_sensor *m_sensor;
	slam_module_ui *m_ui;


	IplImage *canvas;
	float canvas_scale; // px -> mm

	// sonar elevation map
	int elevation;
	int initial_height;

private:
	bool running;
};

