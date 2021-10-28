#include "hough_thread.h"
#include "pnp_thread.h"

HoughThread::HoughThread(int hough_map_x,int hough_map_y, float zone_x, float zone_y, float threshold,int camera_x,int camera_y, int pc_exp_range) { // @suppress("Class members should be properly initialized")
	//sharedPrint("Initialisation of Hough Thread");
	this->m_thread = std::thread();
	this->m_last_input_event_timestamp = 0;

	this->m_tracking = false;
	this->m_hough_map = new float*[this->m_hough_map_x];
	for(int i = 0; i<this->m_hough_map_x; i++)
	{
		this->m_hough_map[i] = new float[this->m_hough_map_y];
		for(int j = 0; j < this->m_hough_map_y; j++)
		{
			this->m_hough_map[i][j] = 0.0;
		}
	}
	this->m_hough_map_baf = new unsigned int*[this->m_hough_map_x];
	for(int i = 0; i<this->m_hough_map_x; i++)
	{
		this->m_hough_map_baf[i] = new unsigned int[this->m_hough_map_y];
		for(int j = 0; j < this->m_hough_map_y; j++)
		{
			this->m_hough_map_baf[i][j] = 0;
		}
	}
	this->m_hough_time_map = new unsigned int*[this->m_hough_map_x];
	for(int i = 0; i<this->m_hough_map_x; i++)
	{
		this->m_hough_time_map[i] = new unsigned int[this->m_hough_map_y];
		for(int j = 0; j < this->m_hough_map_y; j++)
		{
			this->m_hough_time_map[i][j] = 0;
		}
	}
	this->m_look_up_dist = new float**[	this->m_camera_x];
	std::ifstream look_up_file("./look_up.txt");
	if(!look_up_file.is_open())
	{
		this->mutexLog.lock();
		std::cout << "Error: Look-up file not opened" << std::endl;
		this->mutexLog.unlock();
	}
	for(int i = 0; i<	this->m_camera_x; i++)
	{
		this->m_look_up_dist[i] = new float*[	this->m_camera_y];
		for(int j = 0; j<	this->m_camera_y; j++)
		{
			this->m_look_up_dist[i][j] = new float[2];
			if(look_up_file.is_open())
			{
				std::string tmp_input_line;
				do {
					std::getline(look_up_file, tmp_input_line);
				} while(tmp_input_line.at(0) == '#');
				std::string::size_type sz;
				float lu_x = (float)std::stod(tmp_input_line, &sz);
				float lu_y = (float)std::stod(tmp_input_line.substr(sz));
				this->m_look_up_dist[i][j][0] = lu_x;
				this->m_look_up_dist[i][j][1] = lu_y;

			}
			else
			{
				this->m_look_up_dist[i][j][0] = (float)(i);
				this->m_look_up_dist[i][j][1] = (float)(j);
			}
			if(this->m_rho_max-1 < sqrt(this->m_look_up_dist[i][j][0]*this->m_look_up_dist[i][j][0] + this->m_look_up_dist[i][j][1]*this->m_look_up_dist[i][j][1]))
			{
				this->m_rho_max = sqrt(this->m_look_up_dist[i][j][0]*this->m_look_up_dist[i][j][0] + this->m_look_up_dist[i][j][1]*this->m_look_up_dist[i][j][1]) + 1;
			}
		}
	}
	look_up_file.close();
	this->mutexLog.lock();
	std::cout << "Rho max: " << this->m_rho_max << std::endl;
	this->mutexLog.unlock();
	this->m_pc_theta = new float[this->m_hough_map_x];
	this->m_pc_cos = new float[this->m_hough_map_x];
	this->m_pc_sin = new float[this->m_hough_map_x];
	for(int i = 0; i < this->m_hough_map_x; i++)
	{
		this->m_pc_theta[i] = (float)(i)*2*PI/((float)this->m_hough_map_x);
		this->m_pc_cos[i] = cos(this->m_pc_theta[i]);
		this->m_pc_sin[i] = sin(this->m_pc_theta[i]);
	}
	//this->m_decay = 500*1e-6; //200*1e-6
	this->m_pc_exp = new float[this->m_pc_exp_range]; // Déterminer le nombre max de l'exp calculé
	for(unsigned int i = 0; i < this->m_pc_exp_range; i++)
	{
		m_pc_exp[i] = exp(-this->m_decay*(float)(i));
	}
	this->m_pc_hough_coord = new int**[this->m_camera_x];
	for(int i = 0; i < this->m_camera_x; i++)
	{
		this->m_pc_hough_coord[i] = new int*[this->m_camera_y];
		for(int j = 0; j < this->m_camera_y; j++)
		{
			this->m_pc_hough_coord[i][j] = new int[this->m_hough_map_x];
			for(int k = 0;k < this->m_hough_map_x; k++)
			{
				double rho = (double)((int)m_look_up_dist[i][j][0] - (int)(this->m_camera_x >> 1))*this->m_pc_cos[k]+(double)((int)this->m_look_up_dist[i][j][1] - (int)(this->m_camera_y >> 1))*this->m_pc_sin[k];
				int rho_index = (int)round(rho/this->m_rho_max*(double)(this->m_hough_map_y));
				this->m_pc_hough_coord[i][j][k] = rho_index;
			}
		}
	}
	this->m_pc_rho = new float[this->m_hough_map_y];
	for(int i = 0; i < this->m_hough_map_y; i++)
	{
		this->m_pc_rho[i] = (float)(i)/(float)(this->m_hough_map_y)*m_rho_max;
	}
}

void HoughThread::threadFunction() {
	this->m_is_launched = true;
	this->mutexLog.lock();
	std::cout << "Doing my things ! Hough" << std::endl;
	this->mutexLog.unlock();
	std::unique_lock<std::mutex> lck(this->m_main_loop_mutex);
	bool tmp_stop = false;
	std::chrono::steady_clock::time_point begin;
	//std::this_thread::sleep_for(std::chrono::microseconds(2500));
	std::chrono::steady_clock::time_point end;
	unsigned int event_counter = 0;
	do {
		while(this->m_ev_queue.empty())
		{
			/*this->mutexLog.lock();
			std::cout << "Waiting Event" << std::endl;
			this->mutexLog.unlock();*/
			this->m_main_loop_cv.wait(lck);
		}
		auto start = std::chrono::steady_clock::now();
		this->m_ev_add_mutex.lock();
		Event e = this->m_ev_queue.front();
		this->m_ev_queue.pop();
		this->m_ev_add_mutex.unlock();
		switch(e.a)
		{
		case 1:
			if(event_counter++ == 0)
			{
				 begin = std::chrono::steady_clock::now();
			}
			this->computeEvent(e.x,e.y,e.t);
//			this->mutexLog.lock();
//			std::cout << "Adding event " << e.x << " " << e.y << std::endl;
//			this->mutexLog.unlock();
			break;
		case 2:
			tmp_stop = true;
			break;
		default:
			this->mutexLog.lock();
			std::cout << "Unknown Hough event: " << e.a << std::endl;
			this->mutexLog.unlock();
			break;
		}
		auto end = std::chrono::steady_clock::now();
		this->mutexLog.lock();
		std::cout << "Time Hough: " << std::chrono::duration_cast<std::chrono::nanoseconds>(start-end).count() << std::endl;
		this->mutexLog.unlock();
	}while(!tmp_stop);
	end = std::chrono::steady_clock::now();
	this->mutexLog.lock();
	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;
	std::cout << "Time for one event = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count())/(double)(event_counter) <<std::endl;
	this->mutexLog.unlock();
	//this->computeEvent(110,37,0);
	//this->printHoughMap();
	this->mutexLog.lock();
	std::cout << "Stopped my things ! Hough" << std::endl;
	this->mutexLog.unlock();
}


HoughThread::~HoughThread() {

}


int HoughThread::computeEvent(unsigned int x, unsigned int y, unsigned int timestamp)
{
	int nbr_event_generated = 0;
#ifdef ONE_BY_ONE_HOUGH
	if(!this->m_tracking)
	{
		for(int theta_index = 0; theta_index < this->m_hough_map_x; theta_index++)
		{
			int rho_index = this->m_pc_hough_coord[x][y][theta_index];
			if(rho_index < this->m_hough_map_y && rho_index >= 0)
			{
				/*this->mutexLog.lock();
				std::cout << "Time: " <<  timestamp-this->m_hough_time_map[theta_index][rho_index]  <<  timestamp << std::endl;
				this->mutexLog.unlock();*/
				this->m_hough_map[theta_index][rho_index] = this->m_hough_map[theta_index][rho_index]*this->getPCExp(timestamp-this->m_hough_time_map[theta_index][rho_index]) + 1.0;
				if(this->m_hough_map[theta_index][rho_index] >= this->m_threshold)
				{

					/*for(int i = 0; i<this->m_hough_map_x; i++)
					{
						for(int j = 0; j < this->m_hough_map_y; j++)
						{
							this->m_hough_map[i][j] = 0.0;
						}
					}*/
					bool is_peak = true;
					for(int i = -this->m_zone_x; i <= this->m_zone_x; i++)
					{
						if(!is_peak)
						{
							continue;
						}
						for(int j = -this->m_zone_y; j <= this->m_zone_y; j++)
						{
							if(!is_peak || (j == 0 && i == 0))
							{
								continue;
							}
							if(j+rho_index < 0)
							{
								unsigned int index_0 = (unsigned int)((theta_index+i+(this->m_hough_map_x>>1)))%this->m_hough_map_x;
								unsigned int index_1 = -rho_index-j-1;
								if(this->m_hough_map[index_0][index_1] > this->m_hough_map[theta_index][rho_index])
									is_peak = true;
							}
							else
							{
								unsigned int index_0 = (unsigned int)((theta_index+i))%this->m_hough_map_x;
								unsigned int index_1 = rho_index+j;
								if(this->m_hough_map[index_0][index_1] > this->m_hough_map[theta_index][rho_index])
									is_peak = true;
							}
						}
					}
					//this->m_hough_map[theta_index][rho_index] = 0.0;
//					std::future<void> f = std::async(std::launch::async,&PNPThread::addEvent, this->m_pnpt, this->m_pc_theta[theta_index], this->m_pc_rho[rho_index],timestamp,-1);
					if(is_peak)
					{
						this->m_pnpt->addEvent(this->m_pc_theta[theta_index],this->m_pc_rho[rho_index],timestamp,-1);
					}
//					this->mutexLog.lock();
//					std::cout << "Emit event:" << this->m_pc_theta[theta_index] << " " << this->m_pc_rho[rho_index] << " with " << x << " " << y << " " << timestamp << std::endl;
//					this->mutexLog.unlock();
				}
				this->m_hough_time_map[theta_index][rho_index] = timestamp;
			}
		}

	}
	else
	{
		for(int theta_index = 0; theta_index < this->m_hough_map_x; theta_index++)
		{
			int rho_index = this->m_pc_hough_coord[x][y][theta_index];
			if(rho_index < this->m_hough_map_y && rho_index >= 0)
			{
				if(this->m_pnpt->getFilterValue(theta_index,rho_index) <= 0)
				{
					continue;
				}
				this->m_hough_map[theta_index][rho_index] = this->m_hough_map[theta_index][rho_index]*this->getPCExp(timestamp-this->m_hough_time_map[theta_index][rho_index]) + 1.0;
				if(this->m_hough_map[theta_index][rho_index] >= this->m_threshold)
				{
					for(int i = -this->m_zone_x; i <= this->m_zone_x; i++)
					{
						for(int j = -this->m_zone_y; j <= this->m_zone_y; j++)
						{
							if(j+rho_index < 0)
							{
								this->m_hough_map[(unsigned int)((theta_index+i))%this->m_hough_map_x][-rho_index-j-1] = 0.0;
							}
							else
							{
								this->m_hough_map[(unsigned int)((theta_index+i))%this->m_hough_map_x][rho_index+j] = 0.0;
							}
						}
					}
					this->m_pnpt->addEvent(this->m_pc_theta[theta_index],this->m_pc_rho[rho_index],timestamp,this->m_pnpt->getFilterValue(theta_index,rho_index));
				}
				this->m_hough_time_map[theta_index][rho_index] = timestamp;
			}
		}
	}
#else
	const int rho_limit = this->m_hough_map_y-3;
	float dyn_threshold = 0;
	for(int theta_index = 0; theta_index < this->m_hough_map_x; theta_index++)
	{
		int rho_index = this->m_pc_hough_coord[x][y][theta_index];
		if(rho_index < rho_limit && rho_index >= 0)
		{
			unsigned int dt = timestamp-this->m_hough_map_baf[theta_index][rho_index];
			this->m_hough_map[theta_index][rho_index] = this->m_hough_map[theta_index][rho_index]*this->getPCExp(timestamp-this->m_hough_time_map[theta_index][rho_index]) + 1.0;
			this->m_hough_time_map[theta_index][rho_index] = timestamp;
			if(this->m_hough_map[theta_index][rho_index] >= this->m_threshold && dt > 300 && (rho_index != 0 || theta_index < (this->m_hough_map_x>>1)))
			{
				dyn_threshold = this->m_hough_map[theta_index][rho_index];
				unsigned int mod_x = (unsigned int)this->m_hough_map_x;
				unsigned int index_1, index_0;
				{
					//this->mutexLog.lock();
					//std::cout << "Time: " << timestamp << " BAF: " << this->m_hough_map_baf[theta_index][rho_index] << std::endl;
					//std::cout << "TI: " << theta_index << " RI: " << rho_index << " V:" << this->m_hough_map[theta_index][rho_index] << std::endl;
					index_1 = rho_index;
					index_0 = (theta_index-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1++;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1--;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1--;
					if(index_1 > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 1;
						index_0 = ((index_0+(this->m_hough_map_x>>1)))%mod_x;
					}
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;

					// 3x3 filter done;

					dyn_threshold = this->m_hough_map[theta_index][rho_index]*0.9;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+4)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1 = rho_index;
					//index_0 = (theta_index+2)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-4)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1++;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+4)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1++;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1 = rho_index-1;
					if(index_1-- > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 2;
						index_0 = ((index_0+(mod_x>>1)))%mod_x;
					}
					else if(index_1 > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 1;
						index_0 = ((index_0+(mod_x>>1)))%mod_x;
					}
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;

					//5x5 filter done

					dyn_threshold = this->m_hough_map[theta_index][rho_index]*0.8;
					index_0 = (index_0+1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-6)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1 = rho_index;
					//index_0 = (theta_index+3)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+6)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1++;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-6)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1++;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+6)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1++;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1 = rho_index-1;
					if(index_1 > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 1;
						index_0 = ((index_0+(mod_x>>1)))%mod_x;
					}
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0+6)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_1 = rho_index-1;
					if(index_1-- > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 3;
						index_0 = ((index_0+(mod_x>>1)))%mod_x;
					}
					else if(index_1-- > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 2;
						index_0 = ((index_0+(mod_x>>1)))%mod_x;
					}
					else if(index_1 > (unsigned int)(this->m_hough_map_y))
					{
						index_1 = 1;
						index_0 = ((index_0+(mod_x>>1)))%mod_x;
					}
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;
					index_0 = (index_0-1)%mod_x;
					HOUGH_CHECK_PEAK_FUNCTION;

					//7x7 filter done

				}
				//if(dt < 10000)
				this->m_pnpt->addEvent(this->m_pc_theta[theta_index],this->m_pc_rho[rho_index],timestamp,-1);
				this->m_hough_map_baf[theta_index][rho_index] = timestamp;
				//std::cout << "event_sent" << std::endl;
				end_peak_compare_without_tracking:
				//this->mutexLog.unlock();
				continue;
			}
		}
	}
//	}
//	else
//	{
//		for(int theta_index = 0; theta_index < this->m_hough_map_x; theta_index++)
//		{
//			int rho_index = this->m_pc_hough_coord[x][y][theta_index];
//			if(rho_index < this->m_hough_map_y && rho_index >= 0)
//			{
//				unsigned int dt = timestamp-this->m_hough_map_baf[theta_index][rho_index];
//				this->m_hough_map[theta_index][rho_index] = this->m_hough_map[theta_index][rho_index]*this->getPCExp(timestamp-this->m_hough_time_map[theta_index][rho_index]) + 1.0;
//				this->m_hough_time_map[theta_index][rho_index] = timestamp;
//				int line_id = this->m_pnpt->getFilterValue(theta_index,rho_index);
//				if(line_id <= 0)
//				{
//					continue;
//				}
//				if(this->m_hough_map[theta_index][rho_index] >= this->m_threshold && dt > 500)
//				{
//					dyn_threshold = this->m_hough_map[theta_index][rho_index]*0.95;
//					unsigned int mod_x = (unsigned int)this->m_hough_map_x;
//					unsigned int index_1, index_0;
//					{
//						this->mutexLog.lock();
//						std::cout << "Time: " << timestamp << " BAF: " << this->m_hough_map_baf[theta_index][rho_index] << std::endl;
//						std::cout << "TI: " << theta_index << " RI: " << rho_index << std::endl;
//						index_1 = rho_index;
//						index_0 = (theta_index-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1++;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1--;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1--;
//						if(index_1 > (unsigned int)(this->m_hough_map_y))
//						{
//							index_1 = 0;
//							index_0 = ((index_0+(this->m_hough_map_x>>1)))%mod_x;
//						}
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						// 3x3 filter done;
//						dyn_threshold *= 0.7;
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+4)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1 = rho_index;
//						index_0 = (theta_index+2)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-4)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1++;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+4)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1++;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0-1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_1 = rho_index-1;
//						if(index_1-- > (unsigned int)(this->m_hough_map_y))
//						{
//							index_1 = 1;
//							index_0 = ((index_0+(mod_x>>1)))%mod_x;
//						}
//						else if(index_1 > (unsigned int)(this->m_hough_map_y))
//						{
//							index_1 = 0;
//							index_0 = ((index_0+(mod_x>>1)))%mod_x;
//						}
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						index_0 = (index_0+1)%mod_x;
////						std::cout << "I0: " << index_0 << " I1: " << index_1 << std::endl;
//						this->m_hough_map[index_0][index_1] = this->m_hough_map[index_0][index_1]*this->getPCExp(timestamp-this->m_hough_time_map[index_0][index_1]);
//						this->m_hough_time_map[index_0][index_1] = timestamp;
//						if(this->m_hough_map[index_0][index_1] > dyn_threshold)
//						{
//							goto end_peak_compare_with_tracking;
//						}
//						//5x5 filter done
//					}
//					this->m_pnpt->addEvent(this->m_pc_theta[theta_index],this->m_pc_rho[rho_index],timestamp,line_id);
//					this->m_hough_map_baf[theta_index][rho_index] = timestamp;
//					end_peak_compare_with_tracking:
//					this->mutexLog.unlock();
//					continue;
//				}
//			}
//		}
//	}
#endif
//	if(timestamp == 13625559)
//	{
//		this->printHoughMap();
//	}
	this->m_last_input_event_timestamp = timestamp;
	return nbr_event_generated;
}

void HoughThread::printHoughMap()
{
	this->mutexLog.lock();
	for(int i = 0; i < this->m_hough_map_y; i++)
	{
		std::cout << "hough_map_line " << i << " " ;
		for(int j = 0; j < this->m_hough_map_x; j++)
		{
			this->m_hough_map[j][i] = this->m_hough_map[j][i] * this->getPCExp(this->m_last_input_event_timestamp-this->m_hough_time_map[j][i]);
			this->m_hough_time_map[j][i] = this->m_last_input_event_timestamp;
			std::cout << this->m_hough_map[j][i] << " ";
		}
		std::cout << std::endl;
	}
	this->mutexLog.unlock();
}

void HoughThread::lockAddEvent()
{
	this->m_ev_add_mutex.lock();
}

void HoughThread::unlockAddEvent()
{
	this->m_ev_add_mutex.unlock();
}

void HoughThread::sendNotifAddEvent()
{
	this->m_main_loop_cv.notify_all();
}

void HoughThread::addEvent(unsigned int x, unsigned int y, bool p, unsigned int t)
{
	this->m_ev_queue.push(Event(x,y,p,t,1));
}

void HoughThread::stop()
{
	this->m_ev_add_mutex.lock();
	this->m_ev_queue.push(Event(0,0,0,0,2));
	this->m_ev_add_mutex.unlock();
	BaseThread::stop();
}

float HoughThread::getPCExp(unsigned int dt)
{
	if(dt >= this->m_pc_exp_range)
		return 0;
	else
		return this->m_pc_exp[dt];

}

void HoughThread::activateTracking()
{
	this->m_tracking = true;
	//this->m_threshold = 15.0;
	//this->printFilteringMap();
}

void HoughThread::setPNPThread(PNPThread* pnpt)
{
	this->m_pnpt = pnpt;
}

int HoughThread::getMapX()
{
	while(this->m_hough_map_x <= 0);
	return this->m_hough_map_x;
}

int HoughThread::getMapY()
{
	while(this->m_hough_map_y <= 0);
	return this->m_hough_map_y;
}

int HoughThread::getZoneX()
{
	while(this->m_zone_x <= 0);
	return this->m_zone_x;
}

int HoughThread::getZoneY()
{
	while(this->m_zone_y <= 0);
	return this->m_zone_y;
}

float HoughThread::getRhoMax()
{
	while(this->m_rho_max < 0.001);
	return this->m_rho_max;
}

void HoughThread::printFilteringMap()
{
	this->mutexLog.lock();
	for(int i = 0; i < this->m_hough_map_y; i++)
	{
		std::cout << i << ": " ;
		for(int j = 0; j < this->m_hough_map_x; j++)
		{
			std::cout << this->m_pnpt->getFilterValue(j,i) << " ";
		}
		std::cout << std::endl;
	}
	this->mutexLog.unlock();
}

bool HoughThread::BAF(int x, int y, unsigned int t)
{
	unsigned int dt = t-this->m_hough_map_baf[x][y];
	if(dt > 500)
	{
		this->m_hough_map_baf[x][y] = t;
		return true;
	}
	return false;
}
