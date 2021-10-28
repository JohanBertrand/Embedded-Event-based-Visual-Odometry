/*
 * thread_base.h
 *
 *  Created on: 12 juin 2019
 *      Author: Johan
 */

#ifndef BASE_THREAD_H_
#define BASE_THREAD_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "common.h"

class BaseThread
{
protected:
	static std::mutex mutexLog;
	std::condition_variable m_main_loop_cv;
    std::mutex m_main_loop_mutex;
    std::atomic<bool> m_is_launched;
    std::atomic<bool> m_is_started;
    std::atomic<bool> m_stop;
    std::thread m_thread;

public:
    void initThread();
    virtual void threadFunction();
    BaseThread();
    virtual ~BaseThread();
    void start();
    void stop();
    bool isWorking();
};



#endif /* BASE_THREAD_H_ */
