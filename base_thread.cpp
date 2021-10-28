/*
 * thread_base.cpp
 *
 *  Created on: 12 juin 2019
 *      Author: Johan
 */


#include "base_thread.h"

std::mutex BaseThread::mutexLog;

BaseThread::BaseThread() {
	this->mutexLog.lock();
	std::cout << "Base Thread init" << std::endl;
	this->mutexLog.unlock();
	this->m_thread = std::thread();
	this->m_is_started = false;
	this->m_is_launched = false;
	this->m_stop = false;
	//std::unique_lock<std::condition_variable> lck(this->m_main_loop_mutex);
}

void BaseThread::start() {
	this->m_is_started = true;
	this->m_thread = std::thread(&BaseThread::initThread, this);
	//std::cout << std::cout << typeid(this).name() << '\n';
}

void BaseThread::stop() {
	this->mutexLog.lock();
	std::cout << "Call the stop Base Thread function" << std::endl;
	this->mutexLog.unlock();
	this->m_stop = true;
	this->m_main_loop_cv.notify_all();
	this->mutexLog.lock();
	std::cout << "Call the stop Base Thread function1" << std::endl;
	this->mutexLog.unlock();
	while(!m_thread.joinable());
	this->mutexLog.lock();
	std::cout << "Call the stop Base Thread function2" << std::endl;
	this->mutexLog.unlock();
	this->m_thread.join();
	this->mutexLog.lock();
	std::cout << "Call the stop Base Thread function3" << std::endl;
	this->mutexLog.unlock();
	this->m_is_launched = false;
	this->m_is_started = false;
}

void BaseThread::threadFunction(){
	std::cout << "Should not be seen !!" << '\n';
};

BaseThread::~BaseThread() {
	this->mutexLog.lock();
	std::cout << "Delete the Base Thread" << std::endl;
	this->mutexLog.unlock();
	if(this->m_is_launched == true || this->m_is_started == true)
	{
		this->stop();
	}
}

void BaseThread::initThread() {
	this->m_is_launched = true;
	//std::cout << std::cout << typeid(this).name() << '\n';
	this->threadFunction();
}

bool BaseThread::isWorking() {
	return this->m_is_launched & this->m_is_started & !this->m_stop;
}
