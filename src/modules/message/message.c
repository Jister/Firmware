#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>

#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/laser_msg.h>
#include <uORB/topics/sonar.h>

static bool thread_should_exit = false;		/**< message exit flag */
static bool thread_running = false;		/**< message status flag */
static int message_task;				/**< Handle of message task / thread */

/**
 * message management function.
 */
__EXPORT int message_main(int argc, char *argv[]);

/**
 * Mainloop of message.
 */
int message_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	errx(1, "usage: message {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The message app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int message_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("message already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		message_task = px4_task_spawn_cmd("message",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     2000,
					     message_thread_main,
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

int message_thread_main(int argc, char *argv[])
{
	/* subscribe to sensor_combined topic */
		

	int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	//int sensor_combined_fd = orb_subscribe(ORB_ID(sensor_combined));
	int laser_sub_fd = orb_subscribe(ORB_ID(laser_msg));
	int sonar_sub_fd = orb_subscribe(ORB_ID(sonar));
	//int manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
	
	orb_set_interval(attitude_sub_fd,1000);
	//orb_set_interval(sensor_combined_fd,1000);

	orb_set_interval(laser_sub_fd,50);
	orb_set_interval(sonar_sub_fd,50);
	//orb_set_interval(manual_sub_fd,75);
	
	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		
		{.fd = attitude_sub_fd,  .events = POLLIN},		
		/* there could be more file descriptors here, in the form like:
		* { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;


	warnx("[message] starting\n");

	thread_running = true;

	while (!thread_should_exit) {

		
		
		             /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
			int poll_ret = poll(fds, 1, 1000);

			/* handle the poll result */
			if (poll_ret == 0) {
			             /* this means none of our providers is giving us data */
				printf("[message] Got no data within a second\n");
			} else if (poll_ret < 0) {
			             /* this is seriously bad - should be an emergency */
				if (error_counter < 10 || error_counter % 50 == 0) {
				             /* use a counter to prevent flooding (and slowing us down) */
					printf("[message] ERROR return value from poll(): %d\n"
					       , poll_ret);
				}
				error_counter++;
			} else {
				if (fds[0].revents & POLLIN) {
				             /* obtained data for the first file descriptor */
				
					struct vehicle_attitude_s att;	
					//struct sensor_combined_s raw;		
					struct laser_msg_s laser;
					struct sonar_s sonar;
					//struct manual_control_setpoint_s manual;
					
			
					orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &att);
					//orb_copy(ORB_ID(sensor_combined), sensor_combined_fd, &raw);
					orb_copy(ORB_ID(laser_msg), laser_sub_fd, &laser);
					orb_copy(ORB_ID(sonar), sonar_sub_fd, &sonar);
					//orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &manual);
		
					//printf("[message] manual(roll):%8.4f\n",(double)manual.y);
					//printf("[message] manual(pitch):%8.4f\n",(double)manual.x);
					printf("[message] laser_angle:%8.4f\n",(double)laser.laser_angle);
					printf("[message] laser_distance:%8.4f\n\n",(double)laser.laser_distance);
					printf("[message] sonar_F:%d\n",sonar.Front);
					printf("[message] sonar_B:%d\n",sonar.Back);
					printf("[message] sonar_L:%d\n",sonar.Left);
					printf("[message] sonar_R:%d\n",sonar.Right);

					//printf("[message] magnetometer-x:%8.4f\n",(double)raw.magnetometer_raw[0]);
					//printf("[message] magnetometer-y:%8.4f\n",(double)raw.magnetometer_raw[1]);
					//printf("[message] magnetometer-z:%8.4f\n\n",(double)raw.magnetometer_raw[2]);

					
					
				}
			}
		
	}

	warnx("[message] exiting.\n");

	thread_running = false;

	return 0;
}
