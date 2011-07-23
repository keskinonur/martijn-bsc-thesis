#pragma once
#include <windows.h>
#include <fstream>
#include "yaml.h"


// Semaphore
#define MAX_SEM_COUNT 10
#define THREADCOUNT 3


class bot_ardrone;
struct bot_ardrone_measurement;
struct bot_ardrone_control;
struct bot_ardrone_frame;

using namespace std;


class bot_ardrone_recorder
{
public:
	bot_ardrone_recorder(bot_ardrone *bot);
	~bot_ardrone_recorder(void);

	/* record */
	void record_measurement(bot_ardrone_measurement *m);
	void record_control(bot_ardrone_control *c);
	void record_frame(bot_ardrone_frame *f);

	/* playback */
	void playback(char *dataset);

	/* other */
	void prepare_dataset();
	void copy_measurement(bot_ardrone_measurement *from, bot_ardrone_measurement *to);
	void get_last(bot_ardrone_measurement *m);

	bot_ardrone_measurement *last;

private:
	bot_ardrone *bot;


	char dataset_dir[25];
	ifstream fin;
	int frame_counter;

	// resource sharing
	static HANDLE ghSemaphore;
	static HANDLE lastSemaphore;

	/* faster */
	FILE *file_out;
};