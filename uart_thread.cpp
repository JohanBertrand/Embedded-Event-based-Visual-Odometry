#include "uart_thread.h"

UARTThread::UARTThread(unsigned int camera_x, unsigned int camera_y) {
	this->m_ht = 0;
	this->m_fd_rec = -1;
	this->m_fd_command = -1;
	this->m_baf_time = 3000;
	this->m_camera_x = camera_x;
	this->m_camera_y = camera_y;
	this->m_baf_time_array = new unsigned int*[this->m_camera_x>>1];
	for(int i = 0; i < this->m_camera_x>>1; i++)
	{
		this->m_baf_time_array[i] = new unsigned int[this->m_camera_y>>1];
		for(int j = 0; j < this->m_camera_y>>1; j++)
		{
			this->m_baf_time_array[i][j] = 0;
		}
	}
#if MODE == MODE_ONLINE
#if OS == OS_WINDOWS
	this->m_fd_command = RS232_GetPortnr("COM12");
	this->m_fd_rec = this->m_fd_command;
#elif OS == OS_LINUX
	this->m_fd_command = RS232_GetPortnr("ttyUSB0");
	this->m_fd_rec = RS232_GetPortnr("ttyUSB0"); //ttyAMA0
#endif
	RS232_OpenComport(this->m_fd_command,12000000,"8N1",0);
	this->mutexLog.lock();
	std::cout << "File descriptor: " << this->m_fd_command << std::endl;
	this->mutexLog.unlock();
	RS232_cputs(this->m_fd_command, "E-\n");
	//RS232_cputs(this->m_fd, "??\n");
	RS232_cputs(this->m_fd_command, "!U0\n");
	RS232_cputs(this->m_fd_command, "!L2\n");
	RS232_cputs(this->m_fd_command, "!E4\n");
	RS232_cputs(this->m_fd_command, "!B0=54\n");
	RS232_cputs(this->m_fd_command, "!B1=1108364\n");
	RS232_cputs(this->m_fd_command, "!B2=16777215\n");
	RS232_cputs(this->m_fd_command, "!B3=8159221\n");
	RS232_cputs(this->m_fd_command, "!B4=111\n");
	RS232_cputs(this->m_fd_command, "!B5=159147\n");
	RS232_cputs(this->m_fd_command, "!B6=0\n");
	RS232_cputs(this->m_fd_command, "!B7=16777215\n");
	RS232_cputs(this->m_fd_command, "!B8=569762\n");
	RS232_cputs(this->m_fd_command, "!B9=7538\n");
	RS232_cputs(this->m_fd_command, "!B10=51\n");
	RS232_cputs(this->m_fd_command, "!B11=3\n");
	//RS232_cputs(this->m_fd_command, "!U=3000000\n");
	//RS232_CloseComport(this->m_fd_command);
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//RS232_OpenComport(this->m_fd_command,3000000,"8N1",0);
	//if(this->m_fd_command != this->m_fd_rec)
	//{
	//	RS232_OpenComport(this->m_fd_rec,3000000,"8N1",1);
	//}
#endif
	/*libusb_device **devs;
	int r;
	ssize_t cnt;

	this->mutexLog.lock();
	std::cout << "Init: libusb" << std::endl;
	this->mutexLog.unlock();

	r = libusb_init(NULL);
	if (r < 0)
	{
		this->mutexLog.lock();
		std::cout << "Error: libusb initialization" << std::endl;
		this->mutexLog.unlock();
	}
	else
	{
		cnt = libusb_get_device_list(NULL, &devs);
		if (cnt < 0){
			libusb_exit(NULL);
			this->mutexLog.lock();
			std::cout << "Error: libusb device list " << cnt << std::endl;
			this->mutexLog.unlock();
		}
		else
		{
			libusb_device *dev;
			int i = 0, j = 0;
			uint8_t path[8];

			while ((dev = devs[i++]) != NULL) {
				struct libusb_device_descriptor desc;
				int r = libusb_get_device_descriptor(dev, &desc);
				if (r < 0) {
					fprintf(stderr, "failed to get device descriptor");
					return;
				}

				printf("%04x:%04x (bus %d, device %d)",
					desc.idVendor, desc.idProduct,
					libusb_get_bus_number(dev), libusb_get_device_address(dev));

				r = libusb_get_port_numbers(dev, path, sizeof(path));
				if (r > 0) {
					printf(" path: %d", path[0]);
					for (j = 1; j < r; j++)
						printf(".%d", path[j]);
				}
				printf("\n");
			}

			libusb_free_device_list(devs, 1);

			libusb_exit(NULL);
		}
	}*/

	std::cout << "Init ended: RS232" << std::endl;
}

UARTThread::~UARTThread() {

}

void UARTThread::threadFunction() {
	this->m_is_launched = true;
	this->mutexLog.lock();
	std::cout << "Doing my things ! UART" << std::endl;
	this->mutexLog.unlock();
//	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//	for(int i = 0; i < 100000; i++)
//	{
//		this->m_ht->addEvent(112,8,true,1);
//	}
	unsigned int last_time = 0;
	unsigned int first_time = 0;
	unsigned int event_received_global = 0;
	unsigned int event_sended_global = 0;
#if MODE == MODE_OFFLINE
	std::ifstream event_file("./down_58mm.csv");
	if(!event_file.is_open())
	{
		this->mutexLog.lock();
		std::cout << "Error: Event file not opened" << std::endl;
		this->mutexLog.unlock();
	}
	else
	{
		std::string tmp_input_line;
		std::string::size_type sz;
		std::getline(event_file, tmp_input_line); //Remove header of the CSV
		int x,y,p,t;
		int last_t = 0;
		while (std::getline(event_file, tmp_input_line)) {
			x = std::stoi(tmp_input_line, &sz);
			tmp_input_line = tmp_input_line.substr(sz+1);
			y = std::stoi(tmp_input_line, &sz);
			tmp_input_line = tmp_input_line.substr(sz+1);
			p = std::stoi(tmp_input_line, &sz);
			tmp_input_line = tmp_input_line.substr(sz+1);
			t = std::stoi(tmp_input_line);
			if(last_t == 0)
			{
				last_t = t;
			}
			while(t - last_t > 10000)
			{
				last_t = last_t + 10000;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			auto start = std::chrono::steady_clock::now();
			unsigned int y0, x0, p0, c0, t0;
			y0 = y & 0x7f;
			c0 = (y &0x80) >> 7;
			x0 = y & 0x7f;
			p0 = (y &0x80) >> 7;
			t0 = 0;
			t0 += 16777216*y;
			t0 += 65536*y;
			t0 += 256*y;
			t0 += 1*y;
			event_received_global++;
			p = (y0 + c0 + x0 + p0 + t0)%2;
			int tx = x >> 1;
			int ty = y >> 1;

			//std::cout << "Event: " << x << ";" << y << std::endl;
			event_received_global++;
			auto endbaf = std::chrono::steady_clock::now();
			if(t-this->m_baf_time_array[tx][ty] < this->m_baf_time)
			{
				event_sended_global++;
				this->m_ht->lockAddEvent();
				this->m_ht->addEvent(x,y,p==1,t);
				this->m_ht->unlockAddEvent();
				this->m_ht->sendNotifAddEvent();
			}
			if(tx > 0 && tx < (this->m_camera_x>>1)-1 && ty > 0 && ty < (this->m_camera_y>>1)-1)
			{
				this->m_baf_time_array[tx+1][ty+1] = t;
				this->m_baf_time_array[tx  ][ty+1] = t;
				this->m_baf_time_array[tx-1][ty+1] = t;
				this->m_baf_time_array[tx+1][ty  ] = t;
				this->m_baf_time_array[tx-1][ty  ] = t;
				this->m_baf_time_array[tx+1][ty-1] = t;
				this->m_baf_time_array[tx  ][ty-1] = t;
				this->m_baf_time_array[tx-1][ty-1] = t;
			}
			auto end = std::chrono::steady_clock::now();
			this->mutexLog.lock();
			std::cout << "Time BAF: " << std::chrono::duration_cast<std::chrono::nanoseconds>(endbaf-end).count() << std::endl;
			std::cout << "Time UART: " << std::chrono::duration_cast<std::chrono::nanoseconds>(start-end).count() << std::endl;
			this->mutexLog.unlock();
		}
	}
#else
	unsigned char buf[4096];
	unsigned char event_buf[SIZE_BUFFER_EVENT];
	int head = 0;
	int tail = 0;
	int byte_received = 0;
	int event_before_begin = 0;
	bool event_received = false;
	RS232_flushRX(this->m_fd_rec);
	RS232_flushRX(this->m_fd_command);
	RS232_cputs(this->m_fd_command, "E+\n");
	// First 1k events are removed
	while(!this->m_stop && event_before_begin < 1000)
	{
		while(byte_received < 1 && !this->m_stop && event_before_begin < 1000)
		{
			byte_received = RS232_PollComport(this->m_fd_rec, buf, 4095);
		}
		if(byte_received > 0)
		{
			for(int i = 0; i < byte_received; i++)
			{
				head = (head+1)%SIZE_BUFFER_EVENT;
				event_buf[head] = buf[i];
			}
			while((head >= tail && head-tail > 6) || (head < tail && SIZE_BUFFER_EVENT + head - tail > 6))
			{
				unsigned int t;
				t = 0;
				t += 16777216*event_buf[(tail+3)%SIZE_BUFFER_EVENT];
				t += 65536*event_buf[(tail+4)%SIZE_BUFFER_EVENT];
				t += 256*event_buf[(tail+5)%SIZE_BUFFER_EVENT];
				t += 1*event_buf[(tail+6)%SIZE_BUFFER_EVENT];
				last_time = t;
				if(event_before_begin == 0)
					first_time = t;
				event_before_begin++;
				tail = (tail+6)%SIZE_BUFFER_EVENT;
				//std::cout << event_before_begin << std::endl;
			}
			byte_received = 0;
		}
	}
	// Begin of the received and transmitting phase
	while(!this->m_stop)
	{
 		//std::cout << "Waiting data ..." << std::endl;
		while(byte_received < 1 && !this->m_stop)
		{
			byte_received = RS232_PollComport(this->m_fd_rec, buf, 4095);
		}
		if(byte_received > 0)
		{
			for(int i = 0; i < byte_received; i++)
			{
				head = (head+1)%SIZE_BUFFER_EVENT;
				event_buf[head] = buf[i];
			}
			if((head >= tail && head-tail > 6) || (head < tail && SIZE_BUFFER_EVENT + head - tail > 6))
			{
				event_received = true;
				this->m_ht->lockAddEvent();
			}
			while((head >= tail && head-tail > 6) || (head < tail && SIZE_BUFFER_EVENT + head - tail > 6))
			{
				unsigned int y, x, p, c, t;
				y = event_buf[(tail+1)%SIZE_BUFFER_EVENT] & 0x7f;
				c = (event_buf[(tail+1)%SIZE_BUFFER_EVENT] &0x80) >> 7;
				x = event_buf[(tail+2)%SIZE_BUFFER_EVENT] & 0x7f;
				p = (event_buf[(tail+2)%SIZE_BUFFER_EVENT] &0x80) >> 7;
				t = 0;
				t += 16777216*event_buf[(tail+3)%SIZE_BUFFER_EVENT];
				t += 65536*event_buf[(tail+4)%SIZE_BUFFER_EVENT];
				t += 256*event_buf[(tail+5)%SIZE_BUFFER_EVENT];
				t += 1*event_buf[(tail+6)%SIZE_BUFFER_EVENT];
				event_received_global++;
				/*this->mutexLog.lock();
				std::cout << "Received: " << byte_received  << " bytes; Event " << event_received_global << ": " << x << " " << y << " " << p << " " << t << " " << c << std::endl;
				for(int i = 0; i < 6; i++)
				{
					std::cout << (int)(event_buf[(tail+1+i)%SIZE_BUFFER_EVENT]) << " ";
				}
				std::cout << std::endl;
				this->mutexLog.unlock();*/
				if(t < last_time)
				{
				    this->mutexLog.lock();
					std::cout << "Received: " << byte_received  << " bytes; Event " << event_received_global << ": " << x << " " << y << " " << p << " " << t << " " << c << std::endl << "Head: "<< head << " Tail: "<< tail << std::endl;
				    if(c == 1)
				    {
						std::cout << "Error Timestamp " << t << " " << last_time << std::endl;
				    }
				    else
				    {
						std::cout << "Error Timestamp and Control" << std::endl;
				    }
					this->mutexLog.unlock();
				}
				last_time = t;
				tail = (tail+6)%SIZE_BUFFER_EVENT;
				this->BAF(x,y,t);
			}
			if(event_received)
			{
				this->m_ht->unlockAddEvent();
				this->m_ht->sendNotifAddEvent();
			}
			byte_received = 0;
		}
	}
	// Flushing the data left
	RS232_cputs(this->m_fd_command, "E-\n");
	do
	{
		byte_received = RS232_PollComport(this->m_fd_rec, buf, 4095);
		if(byte_received > 0)
		{
			for(int i = 0; i < byte_received; i++)
			{
				head = (head+1)%SIZE_BUFFER_EVENT;
				event_buf[head] = buf[i];
			}
			if((head >= tail && head-tail > 6) || (head < tail && SIZE_BUFFER_EVENT + head - tail > 6))
			{
				event_received = true;
				this->m_ht->lockAddEvent();
			}
			while((head >= tail && head-tail > 6) || (head < tail && SIZE_BUFFER_EVENT + head - tail > 6))
			{
				unsigned int y, x, p, c, t;
				y = event_buf[(tail+1)%SIZE_BUFFER_EVENT] & 0x7f;
				c = (event_buf[(tail+1)%SIZE_BUFFER_EVENT] &0x80) >> 7;
				x = event_buf[(tail+2)%SIZE_BUFFER_EVENT] & 0x7f;
				p = (event_buf[(tail+2)%SIZE_BUFFER_EVENT] &0x80) >> 7;
				t = 0;
				t += 16777216*event_buf[(tail+3)%SIZE_BUFFER_EVENT];
				t += 65536*event_buf[(tail+4)%SIZE_BUFFER_EVENT];
				t += 256*event_buf[(tail+5)%SIZE_BUFFER_EVENT];
				t += 1*event_buf[(tail+6)%SIZE_BUFFER_EVENT];
				event_received_global++;
				/*this->mutexLog.lock();
				std::cout << "AS: Received: " << byte_received  << " bytes; Event " << event_received_global << ": " << x << " " << y << " " << p << " " << t << " " << c << std::endl;
				for(int i = 0; i < 6; i++)
				{
					std::cout << (int)(event_buf[(tail+1+i)%SIZE_BUFFER_EVENT]) << " ";
				}
				std::cout << std::endl;
				this->mutexLog.unlock();*/
				if(t < last_time)
				{
				    this->mutexLog.lock();
				    if(c == 1)
				    {
						std::cout << "Error Timestamp" << t << " " << last_time << std::endl;
				    }
				    else
				    {
						std::cout << "Error Timestamp and Control" << std::endl;
				    }
					this->mutexLog.unlock();
				}
				last_time = t;
				tail = (tail+6)%SIZE_BUFFER_EVENT;
			}
			if(event_received)
			{
				this->m_ht->unlockAddEvent();
				this->m_ht->sendNotifAddEvent();
			}
		}
	}while(byte_received > 0);
	//RS232_cputs(this->m_fd_command, "!U=12000000\n");
	RS232_CloseComport(this->m_fd_command);
	if(this->m_fd_command != this->m_fd_rec)
	{
		RS232_CloseComport(this->m_fd_rec);
	}
#endif
//	std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
	this->mutexLog.lock();
	std::cout << "Event created by UART: " << event_received_global << " sended: " << event_sended_global << std::endl;
	std::cout << "Times: " << last_time << "-" << first_time << "=" << last_time-first_time << std::endl;
//	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() <<std::endl;
	std::cout << "Stopped my things ! UART" << std::endl;
	this->mutexLog.unlock();
}

void UARTThread::setHoughThread(HoughThread* ht)
{
	this->m_ht = ht;
}

void UARTThread::BAF(int x, int y, unsigned int t)
{
	unsigned char tx = x >> 1;
	unsigned char ty = y >> 1;
	if(t-this->m_baf_time_array[tx][ty] < this->m_baf_time && t-this->m_baf_time_array[tx][ty] > 40)
	{
		this->m_ht->addEvent(x,y,1,t);
	}
	if(tx > 0 && tx < (this->m_camera_x>>1)-1 && ty > 0 && ty < (this->m_camera_y>>1)-1 && t-this->m_baf_time_array[tx][ty] > 40)
	{
		this->m_baf_time_array[tx+1][ty+1] = t;
		this->m_baf_time_array[tx  ][ty+1] = t;
		this->m_baf_time_array[tx-1][ty+1] = t;
		this->m_baf_time_array[tx+1][ty  ] = t;
		this->m_baf_time_array[tx-1][ty  ] = t;
		this->m_baf_time_array[tx+1][ty-1] = t;
		this->m_baf_time_array[tx  ][ty-1] = t;
		this->m_baf_time_array[tx-1][ty-1] = t;
	}
}
