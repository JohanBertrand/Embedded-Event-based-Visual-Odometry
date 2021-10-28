#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cmath>
//#include "mingw.thread.h"
#include <thread>
//#include <mutex>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#if OS == OS_LINUX
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <sys/socket.h>
#include <signal.h>
#include <time.h>
#endif
#include "uart_thread.h"
#include "hough_thread.h"
#include "pnp_thread.h"
//#include "user_thread.h"
#include "common.h"
#include "dspic.h"
//using namespace std;

int main(int argc, char *argv[])
{
    unsigned int n = std::thread::hardware_concurrency();
    std::cout << n << " concurrent threads are supported.\n";

    UARTThread *uart_thread_object = new UARTThread();
	HoughThread *hough_thread_object = new HoughThread(256,128);
	PNPThread *pnp_thread_object = new PNPThread(151.0, hough_thread_object);

    hough_thread_object->setPNPThread(pnp_thread_object);
    uart_thread_object->setHoughThread(hough_thread_object);

	pnp_thread_object->start();
	hough_thread_object->start();
	uart_thread_object->start();

	bool stop = false;
#if OS == OS_LINUX
#if SIMULINK_RETURN == 1
#if DSPIC_COM == 1
	DsPIC dspic;
	dspic.async_read(); //flush rx buffer
	dspic.initVarDspic();  //Init PID,odometry,acceleration,speed
	dspic.setVar8(CODE_VAR_MODE_ASSERV,1); // mode asserv vitesse

	dspic.start();  //Start the motors
	dspic.setSpSpeed(0,0,0);
	bool motor_stopped = true;
//	dspic.stop();  //Start the motors
#endif
	struct UDP_data					udp_data;
	for (int i = 0; i < 20; i++ )
		udp_data.mes[i] = 0.1 + i;
	int sockfd;
	char buffer[4];
	char *hello = "Hello from server";
	struct sockaddr_in servaddr, cliaddr;

	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));

	// Filling server information
	servaddr.sin_family    = AF_INET; // IPv4
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(31415);

	// Bind the socket with the server address
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,
			sizeof(servaddr)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	struct sockaddr_in remote;
	remote.sin_family = AF_INET;
	remote.sin_port = htons(31415);
	remote.sin_addr.s_addr = ::inet_addr("10.0.1.56");

	socklen_t addrSize;
	addrSize = sizeof(remote);

	memset(remote.sin_zero, '\0', sizeof(remote.sin_zero));
	std::cout << "messageLength: " << strlen(hello) << std::endl;
	sendto(sockfd, (const char *)hello, strlen(hello), 0, (struct sockaddr *)&remote, addrSize);

	unsigned int len;
	recvfrom(sockfd, (char *)buffer, 4,
				MSG_WAITALL, ( struct sockaddr *) &cliaddr,
				&len);
#endif
#if RPIT_RETURN == 1
	struct addrinfo 				hints;
	struct addrinfo 				*result, *rp;
	int 							sfd, s, i;
	struct sockaddr_storage 		peer_addr;
	socklen_t 						peer_addr_len;
	ssize_t 						nread;
	struct RPIt_socket_mes_struct	mes;
	struct RPIt_socket_con_struct	con;
	struct timespec 				current_time;

	// Clear mes structure

	mes.timestamp = 0;
	for ( i = 0; i < RPIT_SOCKET_MES_N; i++ )
		mes.mes[i] = 0.0;
	mes.magic = RPIT_SOCKET_MAGIC;

	// Clear con structure

	con.magic = 0;
	con.timestamp = 0;
	for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
		con.con[i] = 0.0;

	memset( &hints, 0, sizeof( struct addrinfo ) );
	hints.ai_family = AF_INET;    // Allow IPv4 or IPv6
	hints.ai_socktype = SOCK_DGRAM; // Datagram socket
	hints.ai_flags = AI_PASSIVE;    // For wildcard IP address
	hints.ai_protocol = 0;					// Any protocol
	hints.ai_canonname = NULL;
	hints.ai_addr = NULL;
	hints.ai_next = NULL;
//	hints.sin_addr.s_addr = INADDR_ANY;

	s = getaddrinfo( NULL, RPIT_SOCKET_PORT, &hints, &result );

	if ( s != 0 ) {
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_server: function getaddrinfo returned: %s\n", gai_strerror( s ) );
		funlockfile( stderr );
		exit( EXIT_FAILURE );
	 }

	//
	//getaddrinfo() returns a list of address structures.
	//Try each address until we successfully bind(2).
	//If socket(2) (or bind(2)) fails, we (close the socket
	//and) try the next address.
	//

	for ( rp = result; rp != NULL; rp = rp->ai_next ) {

		std::cout << "IP: " << rp->ai_addr << std::endl;

		sfd = socket( rp->ai_family, rp->ai_socktype, rp->ai_protocol );
		if ( sfd == -1 )
			continue;

		if ( bind( sfd, rp->ai_addr, rp->ai_addrlen ) == 0 )
			break;									// Success

		close( sfd );
	}

	if ( rp == NULL ) {					// No address succeeded
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_server: could not bind. Aborting.\n" );
		funlockfile( stderr );
		exit( EXIT_FAILURE );
	}

	freeaddrinfo( result );
#endif

	//static const char *postthis = "moo mooo moo moo";
	std::string readBuffer;

	/*if(!curl)
	{
		//BaseThread::mutexLog.lock();
		std::cout << "CURL init failed" << std::endl;
		//BaseThread::mutexLog.unlock();
	}
	else
	{
	}*/
#endif
	while(stop == false)
	{
	#if OS == OS_LINUX
	#if SIMULINK_RETURN == 1
			//peer_addr_len = sizeof( struct sockaddr_storage );
			/*nread = recvfrom(	sfd, (char*)&udp_data, sizeof(udp_data), 0,
												(struct sockaddr *)&peer_addr, &peer_addr_len );

			if(udp_data.mes[0] == 1.0)
			{
				stop = true;
			}
			//std::cout << nread << std::endl;

			// Memcopy is faster than socket read: avoid holding the mutex too long

			if ( nread == -1 )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: function recvfrom exited with error.\n" );
				funlockfile( stderr );

				// Clear control in case of error

				for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
					con.con[i] = 0.0;
			}

			if ( nread != sizeof( struct UDP_data ) )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: function recvfrom did not receive the expected packet size.\n" );
				funlockfile( stderr );

				// Clear control in case of error

				for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
					con.con[i] = 0.0;
			}

			for ( i = 0; i < 10; i++ )
			{
				if(udp_data.mes[i] != 0.0)
				{
					std::cout << "CON" << i << " = " << udp_data.mes[i] << std::endl;
				}
			}*/

			// Critical section : copy of the measurements to a local variable
			//clock_gettime( CLOCK_MONOTONIC, &current_time );

			// Critical section

			//mes.timestamp = (unsigned long long)current_time.tv_sec * 1000000000
			//									+ (unsigned long long)current_time.tv_nsec;

			//memcpy( &local_mes, &mes, sizeof( struct RPIt_socket_mes_struct ) );

			//std::cout << local_mes.mes[5] << std::endl;

			// Send measurements to the socket
			/*double tmp = 0.5;

			if ( sendto(	sfd, (char*)&tmp, sizeof(tmp), 0,
										(struct sockaddr *)&peer_addr,
										peer_addr_len) != sizeof(tmp) )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: error sending measurements.\n" );
				funlockfile( stderr );
			}*/
			recvfrom(sockfd, (char*)&udp_data, sizeof(udp_data), 0, (struct sockaddr *)&cliaddr, &len );
			for (int i = 0; i < 10; i++ )
			{
				if(udp_data.mes[i] != 0.0)
				{
//					std::cout << "CON" << i << " = " << udp_data.mes[i] << std::endl;
				}
			}
			if(udp_data.mes[0] == 1.0)
			{
				stop = true;
			}
#if DSPIC_COM == 1
			if(udp_data.mes[1] == 0 && udp_data.mes[2] == 0 && udp_data.mes[3] == 0 && motor_stopped == false )
			{
				motor_stopped = true;
				dspic.setSpSpeed(0,0,0);
//				dspic.stop();  //Start the motors
			}
			else if(motor_stopped == true && !(udp_data.mes[1] == 0 && udp_data.mes[2] == 0 && udp_data.mes[3] == 0))
			{
				motor_stopped = false;
//				dspic.start();  //Start the motors
				dspic.setSpSpeed(udp_data.mes[1],udp_data.mes[2],udp_data.mes[3]);
			}
			else
			{
				dspic.setSpSpeed(udp_data.mes[1],udp_data.mes[2],udp_data.mes[3]);
			}
#endif
			//sendto(sockfd, (char *)&udp_data, sizeof(udp_data), 0, (struct sockaddr *)&remote, addrSize);
			pnp_thread_object->sendToMatLAB(sockfd, remote,addrSize);
			//std::cout << "Sended" << std::endl;
	#endif
	#if RPIT_RETURN == 1
			peer_addr_len = sizeof( struct sockaddr_storage );
			nread = recvfrom(	sfd, (char*)&con, sizeof( struct RPIt_socket_con_struct ), 0,
												(struct sockaddr *)&peer_addr, &peer_addr_len );

			if ( nread == -1 )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: function recvfrom exited with error.\n" );
				funlockfile( stderr );

				/* Clear control in case of error */

				for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
					con.con[i] = 0.0;
			}

			if ( nread != sizeof( struct RPIt_socket_con_struct ) )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: function recvfrom did not receive the expected packet size.\n" );
				funlockfile( stderr );

				/* Clear control in case of error */

				for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
					con.con[i] = 0.0;
			}

			if ( con.magic != RPIT_SOCKET_MAGIC )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: magic number problem. Expected %d but received %d.\n", RPIT_SOCKET_MAGIC, con.magic );
				funlockfile( stderr );

				/* Clear control in case of error */

				for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
					con.con[i] = 0.0;
			}

			/*if ( sendto(	sfd, (char*)&mes, sizeof( struct RPIt_socket_mes_struct ), 0,
										(struct sockaddr *)&peer_addr,
										peer_addr_len) != sizeof( struct RPIt_socket_mes_struct ) )	{
				flockfile( stderr );
				fprintf( stderr, "rpit_socket_server: error sending measurements.\n" );
				funlockfile( stderr );
			}*/
			pnp_thread_object->sendToRPIT(sfd, peer_addr,peer_addr_len);
	#endif
		//BaseThread::mutexLog.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		//break;
	#elif OS == OS_WINDOWS
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		break;
	#endif
	}
#if DSPIC_COM == 1
	dspic.setSpSpeed(0,0,0);
	dspic.stop();
#endif
	std::cout << "Stopped objects " << std::endl;
	uart_thread_object->stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout << "Stopped objects " << std::endl;
	hough_thread_object->stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout << "Stopped objects " << std::endl;
	pnp_thread_object->stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::cout << "Stopped objects " << std::endl;
	hough_thread_object->printHoughMap();
	delete uart_thread_object;
	delete hough_thread_object;
	delete pnp_thread_object;

    return EXIT_SUCCESS;
}
