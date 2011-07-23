#include "global.h"
#include "slam_module_sensor.h"
#include "bot_ardrone.h"

using namespace cv;


slam_module_sensor::slam_module_sensor(slam *controller):
	KF(3, 3, 0),
	state(3, 1, CV_32F),
	processNoise(3, 1, CV_32F)
{
	this->controller = controller;

	prev_update = clock();

	// measure accel
	measurement = Mat::zeros(3, 1, CV_32F);

	//randn( state, Scalar::all(0), Scalar::all(0.1) );
	KF.transitionMatrix = *(Mat_<float>(3, 3) << 1, 1, 0, 0, 1, 1, 0, 0, 1);

	//setIdentity(KF.measurementMatrix);
	KF.measurementMatrix = *(Mat_<float>(3, 3) << 1, 1, 0, 0, 1, 1, 0, 0, 1);

	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));
	setIdentity(KF.errorCovPost, Scalar::all(0.4));

	randn(state, Scalar::all(0), Scalar::all(0.01) );
	randn(KF.statePost, Scalar::all(0), Scalar::all(0.01));
}


slam_module_sensor::~slam_module_sensor(void)
{
}


void slam_module_sensor::process(bot_ardrone_measurement *m)
{


	/* update transition matrix */
	float difftime = float(((double)clock() - prev_update) / CLOCKS_PER_SEC);
	KF.transitionMatrix.at<float>(0, 1) = difftime;
	KF.transitionMatrix.at<float>(1, 2) = difftime;

	KF.measurementMatrix.at<float>(0, 1) = difftime;
	KF.measurementMatrix.at<float>(1, 2) = difftime;

	prev_update = clock();



	Mat prediction = KF.predict();


	// generate measurement
	state.at<float>(2) = m->accel[0] * MG_TO_MS2;
	measurement = KF.measurementMatrix*state;

	KF.correct(measurement);


	randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
	state = KF.transitionMatrix*state + processNoise;


	printf("state: [%f, %f, %f]\n", state.at<float>(0), state.at<float>(1), state.at<float>(2));
	printf("gt: [%f]\n", m->gt_loc[0]);



	//if (frame == NULL) // we need the frame size in order to calculate the canvas scale
	//	return;

	/*
	// initial height
	if (initial_height == -1)
	{
		initial_height = m->altitude;
		canvas_scale = 2.0f * tan(((BOT_ARDRONE_CAM_FOV)/180.0f)*PI) * (float)initial_height;
		//canvas_scale /= (float)frame->width;
		printf("MAP SCALE: 1px is %f mm\n", canvas_scale);
		printf("MAP SIZE: %f x %f m\n", canvas_scale * 0.8f, canvas_scale * 0.8f);
	}

	elevation = initial_height - m->altitude;
	double rel_elevation = (double)elevation / (double)initial_height;

	if (abs(rel_elevation) > 0.1)
	{
		float a = (2.0f * tan(((BOT_ARDRONE_SONAR_FOV)/180.0f)*PI) * (float) m->altitude);
		// size (mm) to pixels
		a *= 1.0f / canvas_scale;

		if (SLAM_BUILD_OBSTACLE_MAP)
		{
			printf("obstacle found. Distance: %i\n", m->altitude);

			int d = max(1, int(a * 0.5));
			int x, y, w, h;
			x = max(0, last_loc[0] - d);
			y = max(0, last_loc[1] - d);
			w = min(2*d, obstacle_map.rows - x);
			h = min(2*d, obstacle_map.cols - y);

			y = max(0, y - 40); // tmp
			x = max(0, x - 20);
			//y -= 90;

			printf("ROI: %i, %i, %i, %i\n", x, y, w, h);

			Rect r(x, y, w, h);
			Mat roi(obstacle_map, r);
			roi = 0;
		}
	}
	*/

	//int t[3] = {last_loc[0], last_loc[1], elevation};
	//matlab->add_elevation_map_tuple((int*) t);
}