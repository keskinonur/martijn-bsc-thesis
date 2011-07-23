#include "global.h"
#include "slam_module_frame.h"
#include "bot_ardrone.h"

#include "opencv_helpers.h"
#include <cv.hpp>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;


slam_module_frame::slam_module_frame(slam *controller)
{
	this->controller = controller;

	fd = new SurfFeatureDetector(SLAM_SURF_HESSIANTHRESHOLD, 3, 4);
	de = new SurfDescriptorExtractor();

	frame = NULL;
	prev_frame_descriptors = NULL;

	last_loc[0] = 300;
	last_loc[1] = 300;

	dropped_frame_counter = 0;
	frame_counter = 0;
	feature_distance = 0.0;

	prev_frame_h = Mat::eye(3, 3, CV_64F);
	double* h_data = (double*) prev_frame_h.data;
	h_data[2] = 300.0;
	h_data[5] = 300.0;

	if (SLAM_BUILD_OBSTACLE_MAP)
	{
		obstacle_map = Mat(800, 800, CV_8UC1);
		obstacle_map = 255;
	}
}


slam_module_frame::~slam_module_frame(void)
{
}

void slam_module_frame::process(bot_ardrone_frame *f)
{
	if (frame == NULL)
	{
		unsigned short w, h;

		memcpy_s(&w, 2, &f->data[0], 2);
		memcpy_s(&h, 2, &f->data[2], 2);

		w = htons(w);
		h = htons(h);

		frame = cvCreateImageHeader(cvSize(w, h), IPL_DEPTH_8U, 3);
		gray = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	}

	frame->imageData = &f->data[4];


	/* display image */
	//add_noise(frame);
	/*
	add_noise(frame);
	imshow("Image:", frame);
	cvWaitKey(4);
	return;
	*/
	/**/


	// frames from the real ardrone are received in RGB order instead of BGR
	if (!f->usarsim)
		cvCvtColor( frame, frame, CV_RGB2BGR );


	/* find features */
	vector<cv::KeyPoint> keypoints;
	int features_found = find_features(frame, keypoints);


	/* calculate descriptors (on greyscale image) */
	Mat descriptors;
	cvCvtColor(frame, gray, CV_RGB2GRAY);
    de->compute(gray, keypoints, descriptors);


	/* plot keypoints */
	/*
	for (int i = 0; i < (int)keypoints.size(); i++)
	{
		int x = (int)keypoints.at(i).pt.x - 2;
		int y = (int)keypoints.at(i).pt.y - 2;

		for (int x2 = 0; x2 < 5; x2++)
		{
			for (int y2 = 0; y2 < 5; y2++)
			{
				frame->imageData[((y + y2) * frame->width * 3) + (x + x2) * 3] = (unsigned char) 0;
				frame->imageData[((y + y2) * frame->width * 3) + (x + x2) * 3 + 1] = (unsigned char) 0;
				frame->imageData[((y + y2) * frame->width * 3) + (x + x2) * 3 + 2] = (unsigned char) 255;
			}
		}
	}
	*/
	/**/


	/* match with previous frame */
	if (frame_counter > 0)
	{
		double ransacReprojThreshold = 3.0;

		/* new method */
		/*
		prev_frame_keypoints.clear();
		prev_frame_descriptors.empty();

		set_canvas_mask();
		find_features(canvas, prev_frame_keypoints, true);
		IplImage *gray2 = cvCreateImage(cvSize(600, 600), IPL_DEPTH_8U, 1);
		cvCvtColor(canvas, gray2, CV_RGB2GRAY);
		de->compute(gray2, prev_frame_keypoints, prev_frame_descriptors);
		*/
		/*****/


		if (keypoints.size() < 20)
		{
			printf("Not enough features found: dropping frame\n");
			return;
		}

		vector<DMatch> matches;
		dm.match(descriptors, prev_frame_descriptors, matches);

		if (matches.size() < 20)
			return;


		/* calculate transformation (RANSAC) */
		vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
		for( size_t i = 0; i < matches.size(); i++ )
		{
			queryIdxs[i] = matches[i].queryIdx;
			trainIdxs[i] = matches[i].trainIdx;
		}
	

		vector<Point2f> points1;
		KeyPoint::convert(keypoints, points1, queryIdxs);

		vector<Point2f> points2;
		KeyPoint::convert(prev_frame_keypoints, points2, trainIdxs);

		Mat homography = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );


		/* filter method #1: count number of outliers */
		int outliers = 0;
		double d;
		double d_sum = 0.0;

        Mat points3t; perspectiveTransform(Mat(points1), points3t, homography);
        for( size_t i1 = 0; i1 < points1.size(); i1++ )
        {
			d = norm(points2[i1] - points3t.at<Point2f>((int)i1,0));
			d_sum += d;

            if(d > 4 ) // inlier
               outliers++;
        }

		//printf("Average feature distance: %f\n", feature_distance / frame_counter);

		double outliers_percentage =  ((double)outliers / (double)points1.size()) * 100.0;

		//printf("percentage of outers: %f \n", outliers_percentage);

		if (outliers_percentage > 85.0)
		{
			printf("dropped frame based on outliers\n");
			dropped_frame_counter++;
			return;
		}

		Mat absolute_homography = prev_frame_h * homography;
		//Mat absolute_homography = homography;


		/* filter method #2: relative change */
		Mat rel_change = prev_frame_h / absolute_homography;
		//dumpMatrix(absolute_homography);
		Mat tmp = abs(rel_change);
		double min_change = MatMin(tmp);
		double max_change = MatMax(tmp);

		//printf("max_change: %f\n", max_change);

		CvScalar rel_change_avg = mean(tmp);
		//printf("rel change: %f\n", rel_change_avg.val[0]);


		if (abs(absolute_homography.at<double> (0, 0)) > 4.0 || absolute_homography.at<double> (0, 0) < -3.0
			||
			abs(absolute_homography.at<double> (0, 1)) > 4.0 || absolute_homography.at<double> (0, 1) < -3.0
			||
			abs(absolute_homography.at<double> (1, 0)) > 4.0 || absolute_homography.at<double> (1, 0) < -3.0
			||
			abs(absolute_homography.at<double> (1, 1)) > 4.0 || absolute_homography.at<double> (1, 1) < -3.0
			)
		{
			printf("dropped frame based on relative changes\n");
			dropped_frame_counter++;
			return;
		}

		/*
		if (max_change > 2.0 || MatNegCount(rel_change) >= 3)
		{
			printf("dropped frame based on relative changes\n");
			dropped_frame_counter++;
			return;
		}
		*/

		//prev_frame_h *= homography;
		prev_frame_h = absolute_homography;

		//printf("current pos: %i, %i\n", last_loc[0], last_loc[1]);

		feature_distance += (d_sum) / (double)points1.size();
	}


	/* get center position */
	double center[] = { (double)frame->width * 0.5, (double)frame->height * 0.5, 0.0 };
	Mat point_center = Mat(1, 3, CV_64F, center);

	Mat tc = point_center * prev_frame_h;

	double *tc_data = (double*) tc.data;
	double *h_data = (double*) prev_frame_h.data;

	last_loc[0] = int(tc_data[0] + h_data[2]);
	last_loc[1] = int(tc_data[1] + h_data[5]);


	/* draw image */
	CvMat CvHomography = prev_frame_h;
	cvWarpPerspective(frame, controller->canvas, &CvHomography, CV_INTER_LINEAR);


	/* store current frame as previous frame */
	prev_frame_keypoints = keypoints;
	prev_frame_descriptors = descriptors;

	//imshow("Image:", controller->canvas);
	//cvWaitKey(4);
	/*
	if (frame_counter > 0 && frame_counter % 4 == 0)
	{
	IplImage *canvas_resized = cvCreateImage( cvSize(800,800), 8, 3 );
	cvResize(canvas, canvas_resized);
	imshow("Image:", canvas_resized);
	cvWaitKey(4);
	}
	*/

	frame_counter++;

	//printf("Average number features/frame: %f\n", (double)feature_counter / (double)frame_counter);
	//printf("Average features distance: %f\n", feature_distance / (double)frame_counter);
	//Sleep(250);
}


void slam_module_frame::process(IplImage *i)
{
	bot_ardrone_frame *f1 = new bot_ardrone_frame;

	int datasize = i->width * i->height * 3;
	char data[999999];

	unsigned short w, h;
	w = htons(i->width);
	h = htons(i->height);

	memcpy(&data[0], &w, 2);
	memcpy(&data[2], &h, 2);
	memcpy(&data[4], i->imageData, datasize);

	f1->data = data;

	process(f1);
}


int slam_module_frame::find_features(IplImage *img, vector<cv::KeyPoint> &v)
{
	if (SLAM_USE_OBSTACLE_MASK)
		calculate_frame_mask(img->width, img->height);

	// frame_mask is ignored when empty
	if (SLAM_USE_OBSTACLE_MASK)
		fd->detect(img, v, frame_mask);
	else
		fd->detect(img, v);

	//printf("found %i features\n", v.size());

	return v.size();
}


void slam_module_frame::calculate_frame_mask(int width, int height)
{
	IplImage *mask_img = cvCreateImage(cvSize(width, height), 8, 1);
	Mat h_inverse = prev_frame_h.inv();
	CvMat invHomography = h_inverse;

	IplImage *obstacle_map_img = cvCreateImageHeader(cvSize(800, 800), 8, 1);
	obstacle_map_img->imageData = (char*)obstacle_map.data;

	cvWarpPerspective(obstacle_map_img, mask_img, &invHomography, CV_INTER_LINEAR);

	frame_mask = Mat(mask_img, true); // copy

	imshow("Mask:", mask_img);
	cvWaitKey(4);
}

void slam_module_frame::add_noise(IplImage *img)
{

	unsigned char* imagedata = (unsigned char*) img->imageData;
	unsigned char tmp;
	double value;
	unsigned int sum;

	double brightness = 0.0;
	double contrast = 0.0;

	double contrast_random;

	contrast_random = 0.4 + (0.5 * rand() / RAND_MAX);

	if (((double)rand() / (double)RAND_MAX) > 0.5)
		brightness = 0.12 * ((double)rand() / (double)RAND_MAX);
	else
		brightness = -0.12 * ((double)rand() / (double)RAND_MAX);


	for (int i = 0; i < img->width * img->height; i++)
	{
		sum = 0;

		for (int j = 0; j < 3; j++)
		{
			sum += imagedata[i * 3 + j];
		}


		for (int j = 0; j < 3; j++)
		{
			tmp = (unsigned char)imagedata[i * 3 + j];
			value = (double)tmp / 255.0;

			if (brightness < 0.0)
				value = value * ( 1.0 + brightness);
            else
				value = value + ((1 - value) * brightness);

			value = (value - 0.5) * (tan ((contrast + 1) * PI/4) ) + 0.5;

			if (sum > 250)
				value = value + (1.0 - value) * contrast_random;

			imagedata[i * 3 + j] = unsigned char(value * 255.0);
		}
	}



	return;

}