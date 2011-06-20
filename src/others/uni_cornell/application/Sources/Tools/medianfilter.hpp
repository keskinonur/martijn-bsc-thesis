/***********************************************
 * Median Filter
 * Author: Cooper Bills (csb88@cornell.edu)
 * (Thread safe)
 ***********************************************/

#ifndef MEDIANFILTER_HPP
#define MEDIANFILTER_HPP

#include <pthread.h>

class MedianFilter
{
	private:
    double *data;
    int *workspace;
    int size;
    int currentMedian;
    int nextInsert;
    pthread_mutex_t mutex;
    int findMaxIndexRefrence(double *referenced, int *array, int size);
    
	public:
	  MedianFilter(int size, double initVal);
	  ~MedianFilter() {}
	  double getMedian();
	  void pushValue(double val);
};

#endif
