#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <math.h>
#include <termios.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <board_config.h>
#include <uORB/topics/laser_msg.h>
#include <uORB/topics/sonar.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

// designated SERIAL4/5 on Pixhawk
#define SERIAL_PORT		"/dev/ttyS6"
#define MAXSIZE 1000

static bool thread_should_exit = false;		/**< serial_test exit flag */
static bool thread_running = false;		/**< serial_test status flag */
static int serial_task;				/**< Handle of serial_test task / thread */

int  _serial_fd;
char readbuf[50];

char ringbuf[MAXSIZE];     
int read_addr=0;  
int write_addr=0;  

int angle = 0;
int distance = 30000;
int Front = 0;
int Back = 0;
int Left = 0;
int Right = 0;

extern "C" __EXPORT int serial_test_main(int argc, char *argv[]);
int serial_test_thread_main(int argc, char *argv[]);
void serial_init();
int read();
int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop);
static void usage(const char *reason);

int next_data_handle(int addr);
void write_data(char data);
void read_data();

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: serial_test {start|stop|status} [-p <additional params>]\n\n");
}

/*ringbuffer*/  
int next_data_handle(int addr)     
{     
	return (addr+1) == MAXSIZE ? 0:(addr+1) ;     
}     
    
void write_data(char data)  
{  
	*(ringbuf+write_addr) = data;  
	write_addr = next_data_handle(write_addr);  
}  
  
void read_data()  
{
	if(ringbuf[read_addr] == 'M'&&ringbuf[read_addr+18]== '\n')
	{
		angle = 100*(ringbuf[read_addr+1]-48) + 10*(ringbuf[read_addr+2]-48) + (ringbuf[read_addr+3]-48);
		distance = 10000*(ringbuf[read_addr+4]-48) + 1000*(ringbuf[read_addr+5]-48) + 100*(ringbuf[read_addr+6]-48)+ 
		                    10*(ringbuf[read_addr+7]-48) + (ringbuf[read_addr+8]-48);
		Back = 100*(ringbuf[read_addr+9]-48) + 10*(ringbuf[read_addr+10]-48)+(ringbuf[read_addr+11]-48);
		Left = 100*(ringbuf[read_addr+12]-48) + 10*(ringbuf[read_addr+13]-48)+(ringbuf[read_addr+14]-48);
		Right = 100*(ringbuf[read_addr+15]-48) + 10*(ringbuf[read_addr+16]-48)+(ringbuf[read_addr+17]-48);

		for(int i=0 ; i<19; i++)
		{
			read_addr = next_data_handle(read_addr);  
		}
	}else
	{
		for(int i=0 ; i<19; i++)
		{
			read_addr = next_data_handle(read_addr);  
			if(ringbuf[read_addr] == 'M')
			{
				break;
			}
		}
	}	
}  	

/*ringbuffer*/  

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

	newtio.c_cc[VTIME]  = 100;//重要  
	newtio.c_cc[VMIN] = 0;//返回的最小值  重要  
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		perror("com set error");  
		return -1;  
	}  
	return 0;  
}  

void serial_init(){
	_serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
	warnx("serial open: %d",_serial_fd);
	if (_serial_fd < 0) {
		warnx("FAIL: serial fd");
		exit(0);
	}
}

int read()
{
	int ret ;
	
	memset(readbuf,0,50);
	ret = read(_serial_fd, readbuf, 19);
	if (ret < 0) {
		warnx("read err: %d\n", ret);
		return -1;
	}	
	else
		return 0;

}


int serial_test_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("serial_test already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		serial_task = px4_task_spawn_cmd("serial_test",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     2000,
					     serial_test_thread_main,
					     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int serial_test_thread_main(int argc, char *argv[])
{
                            int ret;
                            
	warnx("[serial_test] starting\n");
	thread_running = true;

	/* advertise laser topic */
	struct laser_msg_s laser;
	struct sonar_s sonar;
	memset(&laser, 0, sizeof(laser));
	memset(&sonar, 0, sizeof(sonar));
	orb_advert_t _laser_pub = orb_advertise(ORB_ID(laser_msg), &laser);
	orb_advert_t _sonar_pub = orb_advertise(ORB_ID(sonar), &sonar);
                          
                            serial_init();
                            ret = set_serial(_serial_fd,115200,8,'N',1);
                            if(ret == -1)
                            {
                            	warnx("set error:%d\n",ret);
                            	exit(0);
                            }

	while (!thread_should_exit) {
		angle = 0;
		distance = 30000;
		Back = 0;
		Left = 0;
		Right = 0;

	                           ret = read();
	                           if(ret == -1)
	                           {
	                             	laser.laser_distance = distance;
	                                                        laser.laser_angle = angle;
	                                                        sonar.Front = 0;
	                                                        sonar.Back = 0;
	                                                        sonar.Left = 0;
	                                                        sonar.Right = 0;

	                                                        orb_publish(ORB_ID(laser_msg), _laser_pub, &laser);
	                                                        orb_publish(ORB_ID(sonar), _sonar_pub, &sonar);

	                                                        printf("angle:%d\n", angle);
	                                                        printf("distance:%d\n", distance);
	                                                        printf("Front:%d\n", Front);
	                                                        printf("Back:%d\n", Back);
	                                                        printf("Left:%d\n", Left);
	                                                        printf("Right:%d\n", Right);
	                                                        printf("\n[message] Read:OK\n");
	                           }else
	                           {
	                            	for(int i=0;i<19;i++)
	                           	                            {
                            	                                                    write_data(readbuf[i]);	                         	                        	                     
	                                                        }	                   	
	                                                        read_data();
	                                                        laser.laser_distance = distance;
	                                                        laser.laser_angle = angle;
	                                                        sonar.Front = 0;
	                                                        sonar.Back = Back;
	                                                        sonar.Left = Left;
	                                                        sonar.Right = Right;
	                                                        orb_publish(ORB_ID(laser_msg), _laser_pub, &laser);
	                                                        orb_publish(ORB_ID(sonar), _sonar_pub, &sonar);
	                                                        printf("angle:%d\n", angle);
	                                                        printf("distance:%d\n", distance);
	                                                        printf("Back:%d\n", Back);
	                                                        printf("Left:%d\n", Left);
	                                                        printf("Right:%d\n", Right);
	                                                        printf("\n[message] Read:OK\n\n");
	                           }
	                           
	                            usleep(40000);
	}

	warnx("[serial_test] exiting.\n");
	thread_running = false;
	return 0;
}
