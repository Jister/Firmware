#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <math.h>

#include <poll.h>
#include <nuttx/sched.h>
#include <nuttx/config.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vision_position_estimate.h>

#include <mavlink/mavlink_log.h>

#define SERIAL_PORT	"/dev/ttyS6"
#define MAXSIZE 	80
#define BAUDRATE	115200

static bool thread_should_exit = false;		
static bool thread_running = false;		
static int serial_task;				

int _serial_fd;
int ret;

char ringbuf[MAXSIZE];
char data_buf[20];
char readbuf[40];
int data_transformed[3];
float time_stamp;
float last_x;
float last_y;
float last_z;
float pos_buf[8][2];

int read_addr = 0;  
int write_addr = 0;  

bool valid = true;
bool read_valid = false;


/**
 *  management function.
 */
 extern "C" __EXPORT int serial_main(int argc, char *argv[]);

/**
 * Mainloop .
 */
int serial_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) ;
void serial_init() ;
int next_data_handle(int addr) ;  
int next_data_handle(int addr , int count) ;
void write_data(char data) ;


/**
 * The app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int serial_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("serial already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		serial_task = task_spawn_cmd("serial",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 serial_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int serial_thread_main(int argc, char *argv[])
{
	//int mavlink_fd;
	//mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	warnx("[serial] starting\n");
	thread_running = true;

	/* advertise attitude topic */
	struct vision_position_estimate_s localsense;
	memset(&localsense, 0, sizeof(localsense));
	orb_advert_t pos_pub = orb_advertise(ORB_ID(vision_position_estimate), &localsense);

	serial_init();
	ret = set_serial( _serial_fd, BAUDRATE, 8, 'N', 1 );
        	if(ret == -1)
	{
		warnx("set error: %d\n", ret);
		exit(0);
	}

	while (!thread_should_exit) {
		//warnx("Hello serial!\n");

		read_valid = false;

		ret = read(_serial_fd, readbuf,40);
		if( ret <= 0 )
		{
			warnx("read err: %d\n", ret);

		}else
		{
			for(int i = 0 ; i < 40 ; i++)
			{
				write_data(readbuf[i]) ;
			}
		}

		//Read position data
		for(int i = 0 ; i < MAXSIZE ; i++)
		{
			if((ringbuf[read_addr] == 0xA5) && (ringbuf[next_data_handle(read_addr)] == 0x5A) 
			    && (ringbuf[next_data_handle(read_addr,2)] == 0x2A) && (ringbuf[next_data_handle(read_addr,3)] == 0x21) 
			    && (ringbuf[next_data_handle(read_addr,31)] == 0x55) && (ringbuf[next_data_handle(read_addr,32)] == 0xAA))  
			{
				read_addr = next_data_handle(read_addr,6) ; 
				for(int j = 0 ; j < 20 ; j++)
				{
					data_buf[j] = ringbuf[read_addr] ;
					read_addr = next_data_handle(read_addr) ;
				}
				read_valid = true;
				break;
			}else
			{
				read_addr = next_data_handle(read_addr) ;
			}
		}
		read_addr = next_data_handle(read_addr, 7);
		data_transformed[0] = ((int)data_buf[3]<<24) | ((int)data_buf[2]<<16) | ((int)data_buf[1]<<8) | ((int)data_buf[0]) ;
		data_transformed[1] = ((int)data_buf[7]<<24) | ((int)data_buf[6]<<16) | ((int)data_buf[5]<<8) | ((int)data_buf[4]) ;
		data_transformed[2] = ((int)data_buf[11]<<24) | ((int)data_buf[10]<<16) | ((int)data_buf[9]<<8) | ((int)data_buf[8]) ;
		//time_stamp = ((int)data_buf[19]<<54) | ((int)data_buf[18]<<48) | ((int)data_buf[17]<<40) | ((int)data_buf[16]<<32) | ((int)data_buf[15]<<24) | ((int)data_buf[14]<<16) | ((int)data_buf[13]<<8) | ((int)data_buf[12]) ;
		//printf("x:%d\n",data_transformed[0]);
		// printf("y:%d\n",data_transformed[1]);
		// printf("z:%d\n",data_transformed[2]);

		//if(crc(data_transformed,12) == crc_data)
		//{
		last_x = localsense.x;
		last_y = localsense.y;
		last_z = localsense.z;

		localsense.timestamp_boot = hrt_absolute_time(); 
		localsense.y = data_transformed[0]/100.0f;
		localsense.x = data_transformed[1]/100.0f;
		localsense.z = data_transformed[2]/100.0f;

		if(fabs(localsense.x - last_x) > 10 || fabs(localsense.y - last_y) > 10)
		{
			valid = false;
		}
		if(fabs(localsense.x) > 100 || fabs(localsense.y) > 100)
		{
			valid = false;
		}
		if(fabs(localsense.x - last_x) < 0.0001 && fabs(localsense.y - last_y) < 0.0001)
		{
			valid = false;
		}

		/*
		if(sum_data == last_sum_data){
			valid = false;
		}
		*/
		if(valid && read_valid){
			 // for(int i = 7; i > 0; i--)
			 // {
			 // 	pos_buf[i][0] = pos_buf[i-1][0];
			 // 	pos_buf[i][1] = pos_buf[i-1][1];
			 // }
			 // pos_buf[0][0] = localsense.x;
			 // pos_buf[0][1] = localsense.y;
			 // localsense.x = (pos_buf[7][0] + pos_buf[6][0] + pos_buf[5][0] + pos_buf[4][0] + pos_buf[3][0] + pos_buf[2][0] + pos_buf[1][0] + pos_buf[0][0]) / 8.0f;
			 // localsense.y = (pos_buf[7][1] + pos_buf[6][1] + pos_buf[5][1] + pos_buf[4][1] + pos_buf[3][1] + pos_buf[2][1] + pos_buf[1][1] + pos_buf[0][1]) / 8.0f;
			
			if (pos_pub < 0) {
				pos_pub = orb_advertise(ORB_ID(vision_position_estimate), &localsense);
			} else {
				orb_publish(ORB_ID(vision_position_estimate), pos_pub, &localsense);
				//mavlink_log_info(mavlink_fd, "[localsense] position x:%f  y:%f", (double)localsense.x,(double)localsense.y);
				printf("[localsense] position x:%f  y:%f\n", (double)localsense.x,(double)localsense.y);
			}	
		}else{
			valid = true;
		}	
		//}

		usleep(20000);
	}

	warnx("[serial] exiting.\n");

	thread_running = false;

	return 0;
}

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: serial {start|stop|status} [-p <additional params>]\n\n");
}


int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if  ( tcgetattr( fd,&oldtio)  !=  0) {   
		perror("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
		newtio.c_cflag |= CS7;  
		break;  

		case 8:  
		newtio.c_cflag |= CS8;  
		break;  
	}  

	switch( nEvent )  
	{  
		case 'O':  
		newtio.c_cflag |= PARENB;  
		newtio.c_cflag |= PARODD;  
		newtio.c_iflag |= (INPCK | ISTRIP);  
		break;  

		case 'E':   
		newtio.c_iflag |= (INPCK | ISTRIP);  
		newtio.c_cflag |= PARENB;  
		newtio.c_cflag &= ~PARODD;  
		break;  

		case 'N':    
		newtio.c_cflag &= ~PARENB;  
		break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
		cfsetispeed(&newtio, B2400);  
		cfsetospeed(&newtio, B2400);  
		break;  

		case 4800:  
		cfsetispeed(&newtio, B4800);  
		cfsetospeed(&newtio, B4800);  
		break;

		case 9600:  
		cfsetispeed(&newtio, B9600);  
		cfsetospeed(&newtio, B9600);  
		break;  

		case 19200:  
		cfsetispeed(&newtio, B19200);  
		cfsetospeed(&newtio, B19200);  
		break; 

		case 57600:  
		cfsetispeed(&newtio, B57600);  
		cfsetospeed(&newtio, B57600);  
		break; 

		case 115200:  
		cfsetispeed(&newtio, B115200);  
		cfsetospeed(&newtio, B115200);  
		break;  

		case 460800:  
		cfsetispeed(&newtio, B460800);  
		cfsetospeed(&newtio, B460800);  
		break;  

		default:  
		cfsetispeed(&newtio, B9600);  
		cfsetospeed(&newtio, B9600);  
		break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME] = 10;
	newtio.c_cc[VMIN] = 40;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		perror("com set error");  
		return -1;  
	}  
	return 0;  
}  

void serial_init()
{
	//_serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
	_serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
	warnx("serial open: %d",_serial_fd);
	if (_serial_fd < 0) {
		warnx("FAIL: serial fd");
		exit(0);
	}
}

int next_data_handle(int addr)     
{     
  	return (addr + 1) == MAXSIZE ?  0 : (addr + 1) ;     
}
     
int next_data_handle(int addr , int count)     
{     
  	int a;
  	a = addr;
  	for(int i = 0; i < count ; i++)
  	{ 
    		a = ( (a + 1)  == MAXSIZE ?  0 : ( a + 1 ) ) ;   
  	}
  	return a;  
}
 
void write_data(char data)  
{  
	*(ringbuf+write_addr) = (unsigned char)data;  
	write_addr = next_data_handle(write_addr);  
}  
