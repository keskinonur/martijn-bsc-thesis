#pragma once
#include <windows.h>
#include <fstream>
#include <iostream>
#include <string>

#include "linalg.h"
#include "rl_forcefield.h"
#include "rl_stage.h"

class bot_ardrone;
struct bot_ardrone_measurement;
struct bot_ardrone_control;
struct bot_ardrone_frame;

using namespace std;

static bot_ardrone *rl_bot;


// structures
typedef struct rl_particle {
	double loc[2];
	double vect[2];
	double val;
	struct rl_particle *top;
	struct rl_particle *right;
	struct rl_particle *bottom;
	struct rl_particle *left;
} rl_particle;

typedef struct rl_layout {
	double width;
	double height;
	double offset[2];
	double pylon1[2];
	double pylon2[2];
	double pylonr;
	double coarse;
} rl_layout;

typedef struct rl_transition {
	double loc1[2];
	double loc2[2];
	class rl_stage *stage;
} rl_transition;



class reinflearn
{
public:
	// vars
	rl_stage *stage_curr;
	rl_layout *layout;
	
	// construct/destruct
	reinflearn(bot_ardrone *bot);
	~reinflearn(void);

	// field calculations
	void real2sim(double vect_in[2], double vect_out[2]);
	void sim2real(double vect_in[2], double vect_out[2]);

	// control robot
	void control_set(int type, double velocity);
	void fly_vector(double vect[2]);
	
//private:
	
};