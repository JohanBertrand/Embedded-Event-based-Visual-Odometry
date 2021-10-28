/*
 * common.h
 *
 *  Created on: 4 juin 2019
 *      Author: Johan
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#define PI 3.14159265

#define SIZE_BUFFER_EVENT 8192

#define DEBUG_NO 0
#define DEBUG_YES 1
#define DEBUG_HARD 2

#define OS_WINDOWS 0
#define OS_LINUX 1

#define MODE_ONLINE 0
#define MODE_OFFLINE 1

#define DEBUG DEBUG_NO
#define OS OS_LINUX
#define MODE MODE_OFFLINE
#define SIMULINK_RETURN 0
#define RPIT_RETURN 0
#define DSPIC_COM 0

// RPIT

#if RPIT_RETURN == 1

#define RPIT_SOCKET_CON_N					10			// Nb of double sent (control)
#define RPIT_SOCKET_MES_N					10			// Nb of double returned (measurement)
#define RPIT_SOCKET_PORT					"31415"	// Port of the server
#define RPIT_SOCKET_MES_PERIOD		2000		// Sampling period of the measurement (us)
#define RPIT_SOCKET_MAGIC					3141592	// Magic number
#define RPIT_SOCKET_WATCHDOG_TRIG	1000000	// Delay in us before watchdog is triggered

struct RPIt_socket_mes_struct	{
	unsigned int				magic;							// Magic number
	unsigned long long 	timestamp;					// Absolute server time in ns
	double							mes[RPIT_SOCKET_MES_N];	// Measurements
};

struct RPIt_socket_con_struct	{
	unsigned int				magic;							// Magic number
	unsigned long long 	timestamp;					// Absolute client time in ns
	double							con[RPIT_SOCKET_CON_N];	// Control signals
};

#endif
//END RPIT

struct UDP_data	{
	double							mes[20];
};

class Event {
public:
	unsigned int x;
	unsigned int y;
	bool p;
	unsigned int t;
	unsigned int a;
	Event(unsigned int x, unsigned int y,bool p, unsigned int t,unsigned int a)
	{
		this->x = x;
		this->y = y;
		this->p = p;
		this->t = t;
		this->a = a;
	}
};

class HoughEvent {
public:
	double theta;
	double dist;
	unsigned int t;
	int line_id;
	unsigned int a;
	HoughEvent(double theta, double dist,unsigned int t, int line_id,unsigned int a)
	{
		this->theta = theta;
		this->dist = dist;
		this->t = t;
		this->line_id = line_id;
		this->a = a;
	}
};
//void sharedPrint(std::string msg);

#endif /* COMMON_H_ */
