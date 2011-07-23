#pragma once

#include "bot.h"
#include "botinterface.h"
#include "bot_ardrone_usarsim.h"
#include "bot_ardrone_ardronelib.h"
#include "bot_ardrone_recorder.h"
#include "slam.h"
#include "reinflearn.h"

#include <time.h>

#define BOT_ARDRONE_Velocity 0
#define BOT_ARDRONE_AltitudeVelocity 0
#define BOT_ARDRONE_LinearVelocity 1
#define BOT_ARDRONE_LateralVelocity 2
#define BOT_ARDRONE_RotationalVelocity 3

#define BOT_ARDRONE_INTERFACE_NONE 0
#define BOT_ARDRONE_INTERFACE_USARSIM 1
#define BOT_ARDRONE_INTERFACE_ARDRONELIB 2

#define BOT_ARDRONBOT_EVENT_MEASUREMENT_SEN 0
#define BOT_ARDRONBOT_EVENT_MEASUREMENT_STA 1

#define BOT_ARDRONE_SENSOR_UNKNOW 0
#define BOT_ARDRONE_SENSOR_GT 1
#define BOT_ARDRONE_SENSOR_IMU 2
#define BOT_ARDRONE_SENSOR_SONAR 3
#define BOT_ARDRONE_SENSOR_ACCEL 4


struct bot_ardrone_control {
	double time;
	float velocity[4];
	float velocity_compensate[4];
	bot_state state;

	bot_ardrone_control();
};

struct bot_ardrone_measurement {
	double time;
	int altitude;		// mm

	/* Acceleration: x, y, z in mili-g (x direction = F/B, y direction is L/R, z direction is U/D
	 * measurement from navdata->navdata_phys_measures.phys_accs
	 * 1 g = 9.80665 m/s2 = 32.17405 ft/s2
	 */
	float accel[3];

	/* Orientation: pitch, roll, yaw (milli-degrees)
	 * measured by an angular speed sensor */
	float or[3];

	/* estimated linear velocity: x, y, z (mm/s)
	 * The linear speeds (vx/vy/vz) are given in the local frame
	 * (so you only have to apply a psi rotation to get
	 * them on the ground frame) */
	float vel[3];

	/* USARSim only */
	bool usarsim;
	int type;			// usarsim message type: SEN, NFO, etc
	int sensor;			// temp, i prefer to receive all sensors in one socket message
	float gt_loc[3];
	float gt_or[3];

	bot_ardrone_measurement();
};

struct bot_ardrone_frame {
	double time;
	char *data;
	char *data_start;
	int data_size;
	int dest_size;
	char filename[25];
	bool usarsim;

	bot_ardrone_frame();
};

class bot_ardrone
{
public:
	bot_ardrone(int botinterface);
	~bot_ardrone(void);
	void control_set(int type, int opt, float val);
	float control_get(int type, int opt);
	void control_update();
	void control_update(bot_ardrone_control *c);
	void control_reset();
	void take_off();
	void land();
	void recover(bool send);
	void measurement_received(bot_ardrone_measurement *m);
	void frame_received(bot_ardrone_frame *f);
	static double get_clock();
	void set_record();
	void set_playback(char *dataset);
	void set_laststate();
	void set_slam(bool state);

	static clock_t start_clock;
	int i_id;
	botinterface *i;
	bot_ardrone_control control;
	bot_ardrone_recorder *recorder;
	bool record, playback;
	int battery;		// percentage (0-100%)

	// slam
	clock_t last_measurement_time;
	bool slam_state;
	slam *slamcontroller;
};