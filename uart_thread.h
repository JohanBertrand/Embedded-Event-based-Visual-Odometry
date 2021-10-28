/*
 * uart_thread.h
 *
 *  Created on: 12 juin 2019
 *      Author: Johan
 */

#ifndef UART_THREAD_H_
#define UART_THREAD_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include "common.h"
#include "base_thread.h"
#include "hough_thread.h"
#include "rs232.h"
//#include "libusb.h"

class UARTThread : public BaseThread
{
private:
	HoughThread* m_ht;
	int m_fd_rec;
	int m_fd_command;
	unsigned int** m_baf_time_array;
	unsigned int m_baf_time;
	int m_camera_x;
	int m_camera_y;

	void BAF(int x, int y, unsigned int t);

public:
    UARTThread(unsigned int camera_x = 128, unsigned int camera_y = 128);
    ~UARTThread();
	void threadFunction();
    void setHoughThread(HoughThread* ht);
};

#endif /* UART_THREAD_H_ */
