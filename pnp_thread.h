#ifndef PNP_THREAD_H
#define PNP_THREAD_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <queue>
#include <algorithm>
#include <cmath>
#include "common.h"
#include "base_thread.h"
#include <future>
#include <sstream>
#include <chrono>
#if OS == OS_LINUX
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <time.h>
#endif
//#include "libusb.h"

class HoughThread;

class PNPThread : public BaseThread
{
private:
	const float 		m_focal_length = 151.0;
	float** 	m_epsilon;
	float** 	m_object_points;
	float** 	m_object_matrix;
	int			m_nbr_lines_identified;
	float**	m_line_parameters;
	float**	m_line_inters;
	int**	m_filtering_array;
	const float 		m_confidence_coef = 0.12;
	int** 		m_current_filter_centers;
	int			m_ht_rho_max;
	int 		m_ht_map_x;
	int 		m_ht_map_y;
	std::atomic<float> 	m_posit_z;
	std::atomic<float> 	m_posit_y;
	std::atomic<float>		m_posit_x;
	std::atomic<float>		m_posit_qw;
	std::atomic<float> 	m_posit_qx;
	std::atomic<float>		m_posit_qy;
	std::atomic<float> 	m_posit_qz;
	std::atomic<float> 	m_posit_h;
	std::atomic<float>		m_posit_a;
	std::atomic<float> 	m_posit_b;
	std::atomic<float> 	m_posit_yaw;
	std::atomic<float> 	m_posit_pitch;
	std::atomic<float> 	m_posit_roll;
	std::atomic<float> 	m_posit_m00;
	std::atomic<float> 	m_posit_m01;
	std::atomic<float> 	m_posit_m02;
	std::atomic<float> 	m_posit_m10;
	std::atomic<float> 	m_posit_m11;
	std::atomic<float> 	m_posit_m12;
	std::atomic<float> 	m_posit_m20;
	std::atomic<float> 	m_posit_m21;
	std::atomic<float> 	m_posit_m22;

	std::queue<HoughEvent>	m_ev_queue;
	std::mutex 			m_pose_add_mutex;
	std::mutex 			m_ev_add_mutex;
	std::mutex 			m_filter_mutex;
	HoughThread*		m_ht;

	void multMat(float** m1, float** m2, float** res, int ligne, int inter, int colonne);
	void transMat(float** matrix, float** res, int ligne, int colonne);
	void dispMat(float** m1,int ligne, int colonne);

public:
	void threadFunction();
	PNPThread(float focal_length,HoughThread* ht);
    ~PNPThread();

    void stop();

    void addEvent(float theta, float dist, unsigned int t, int line_id);
    void computeEvent(float theta, float dist, unsigned int t, int line_id);
    void computeLineIntersection();
    void computePosit();
    void updateFilteringArray();
    void updateLineParameters(float theta, float dist, bool rotated, int line_id, bool cycle);
    int getFilterValue(int t, int d);
    void printFilteringMap();
#if OS == OS_LINUX
    void sendToMatLAB(int sockfd, struct sockaddr_in remote, int addr_size);
    void sendToRPIT(int sfd, struct sockaddr_storage peer_addr, int peer_addr_len);
#endif
};

#endif
