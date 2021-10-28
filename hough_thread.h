#ifndef HOUGH_THREAD_H
#define HOUGH_THREAD_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <cmath>
#include <queue>
#include <atomic>
#include <fstream>
#include <future>
#include "common.h"
#include "base_thread.h"

#define HOUGH_CHECK_PEAK_FUNCTION this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);this->m_hough_time_map[index_0][index_1] = timestamp;if(this->m_hough_map[index_0][index_1] > dyn_threshold){goto end_peak_compare_without_tracking;};//std::cout << "I0: " << index_0 << " I1: " << index_1 << " V: " << this->m_hough_map[index_0][index_1] << std::endl;

class PNPThread;

class HoughThread : public BaseThread
{
private:
	const int 	m_hough_map_x = 256;
	const int 	m_hough_map_y = 128;
	const int 	m_camera_x = 128;
	const int 	m_camera_y = 128;
	int 		m_last_input_event_timestamp;
	const unsigned int 	m_pc_exp_range = 1000000;
	const float 			m_threshold = 9.0;
	const float			m_decay = 500e-6;
	float 			m_rho_max;
	const float 			m_zone_x = 5;
	const float 			m_zone_y = 5;
	float** 		m_hough_map;
	unsigned int** 		m_hough_map_baf;
	unsigned int** 		m_hough_time_map;
	float*** 		m_look_up_dist;
	float* 		m_pc_theta;
	float* 		m_pc_cos;
	float* 		m_pc_sin;
	float* 		m_pc_exp;
	float*			m_pc_rho;
	int*** 			m_pc_hough_coord;
	std::atomic<bool>	m_tracking;
	std::queue<Event>	m_ev_queue;
	std::mutex 			m_ev_add_mutex;
	PNPThread*		m_pnpt;


	float getPCExp(unsigned int dt);
	bool BAF(int x, int y, unsigned int t);

public:
	HoughThread(int hough_map_x, int hough_map_y, float zone_x = 5, float zone_y = 5, float threshold = 9.0, int camera_x = 128, int camera_y = 128, int pc_exp_range = 1000000);
    ~HoughThread();

    void stop();
	void threadFunction();

	int computeEvent(unsigned int x, unsigned int y, unsigned int timestamp);
    void printHoughMap();
    void printFilteringMap();
    void lockAddEvent();
    void unlockAddEvent();
    void sendNotifAddEvent();
    void addEvent(unsigned int x, unsigned int y, bool p, unsigned int t);
    void activateTracking();
    void setPNPThread(PNPThread* pnpt);
    int getMapX();
    int getMapY();
    int getZoneX();
    int getZoneY();
    float getRhoMax();
};

#endif
