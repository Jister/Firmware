#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <math.h>
#include <float.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/test.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>

#define PUB_INTERVAL 10000	// limit publish rate to 100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL		// buffer size is 0.5s
#define DELAY 0.40f
#define Average 20

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int inertial_filter_task; /**< Handle of deamon task / thread */

static const hrt_abstime vision_topic_timeout = 500000;	// Vision topic timeout = 0.5s
 
__EXPORT int extended_observer_filter_main(int argc, char *argv[]);
int extended_observer_filter_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
static float fal(float e, float alpha, float delta);
static float sign(float x);

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [-v]\n\n");
	exit(1);
}

static float fal(float e, float alpha, float delta)
{
	if(fabsf(e) > delta)
		return powf(fabsf(e), alpha)*sign(e);
	else
		return e/powf(delta, 1 - alpha);
}

static float sign(float x)
{
	if(x > 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

int extended_observer_filter_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		inertial_filter_task = task_spawn_cmd("inertial_filter",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 5300,
					       extended_observer_filter_thread_main,
					       (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

/****************************************************************************
 * main
 ****************************************************************************/
int extended_observer_filter_thread_main(int argc, char *argv[])
{
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	float x_est[3] = { 0.0f, 0.0f, 0.0f };	// pos, vel
	float y_est[3] = { 0.0f, 0.0f, 0.0f };	// pos, vel
	float z_est[3] = { 0.0f, 0.0f, 0.0f };	// pos, vel

	float est_buf[EST_BUF_SIZE][3][2];	// estimated position buffer
	float R_buf[EST_BUF_SIZE][3][3];	// rotation matrix buffer
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));

	int buf_ptr = 0;

	float x_est_prev[3], y_est_prev[3], z_est_prev[3];
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));

	hrt_abstime accel_timestamp = 0;
	hrt_abstime pub_last = hrt_absolute_time();
	hrt_abstime t_prev = 0;

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame

	float corr_vision[3] = { 0.0f, 0.0f, 0.0f };

	//parameter of extended observer filter
	float beta0 = 1.0f;
	float beta1 = 1.25f;
	float beta2 = 1.8f;
	float delta = 0.02f;

	float vel_buf[Average][2];

	bool vision_valid = false;		// vision is valid
	
	/* declare and safely initialize all structs */
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vision_position_estimate_s vision;
	memset(&vision, 0, sizeof(vision));
	struct test_s test;
	memset(&test, 0, sizeof(test));

	/* subscribe */
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vision_position_estimate_sub = orb_subscribe(ORB_ID(vision_position_estimate));

	/* advertise */
	orb_advert_t predict_position_pub = orb_advertise(ORB_ID(test), &test);

	thread_running = true;

	/* main loop */
	struct pollfd fds[1] ;
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;


	while (!thread_should_exit) {
		int ret = poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* vehicle attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

			bool updated;

			/* sensor combined */
			orb_check(sensor_combined_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.accelerometer_timestamp != accel_timestamp) {
					if (att.R_valid) {
						/* correct accel bias */
						sensor.accelerometer_m_s2[0] -= acc_bias[0];
						sensor.accelerometer_m_s2[1] -= acc_bias[1];
						sensor.accelerometer_m_s2[2] -= acc_bias[2];

						/* transform acceleration vector from body frame to NED frame */
						for (int i = 0; i < 3; i++) {
							acc[i] = 0.0f;

							for (int j = 0; j < 3; j++) {
								acc[i] += PX4_R(att.R, i, j) * sensor.accelerometer_m_s2[j];
							}
						}

						acc[2] += CONSTANTS_ONE_G;

					} else {
						memset(acc, 0, sizeof(acc));
					}

					accel_timestamp = sensor.accelerometer_timestamp;
				}
			}

			orb_check(vision_position_estimate_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(vision_position_estimate), vision_position_estimate_sub, &vision);

				int est_vicon = buf_ptr - 1 - min(EST_BUF_SIZE - 1, max(0, (int)(DELAY * 1000000.0f / PUB_INTERVAL)));

				if (est_vicon < 0) {
					est_vicon += EST_BUF_SIZE;
				}

				/* reset position estimate on first vision update */
				if (!vision_valid) {
					x_est[0] = vision.x;
					y_est[0] = vision.y;

					vision_valid = true;
		
					warnx("Localsense estimate valid");
					mavlink_log_info(mavlink_fd, "[inav] Localsense estimate valid");
				}
				if(fabsf(vision.x - x_est_prev[0]) < 0.0001f && fabsf(vision.y - y_est_prev[0]) < 0.0001f){
					vision_valid = false;
					mavlink_log_info(mavlink_fd, "[inav] Localsense Lost");
				}

				/* calculate correction for position */
				corr_vision[0] = est_buf[est_vicon][0][0] - vision.x ;
				corr_vision[1] = est_buf[est_vicon][1][0] - vision.y ;
				corr_vision[2] = est_buf[est_vicon][2][0] - vision.z ;
			}
		}

		/* check for timeout on vision topic */
		if (vision_valid && (t > (vision.timestamp_boot + vision_topic_timeout))) {
			vision_valid = false;
			warnx("Localsense timeout");
			mavlink_log_info(mavlink_fd, "[inav] Localsense timeout");
		}

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
		t_prev = t;


		if (vision_valid) {
			x_est[2] = x_est_prev[2] - beta2 * fal(corr_vision[0], 0.75, delta);
			x_est[1] = x_est_prev[1] + dt * (x_est_prev[2] + acc[0]) - beta1 * fal(corr_vision[0], 0.5, delta);
			x_est[0] = x_est_prev[0] + dt * x_est_prev[1] - beta0 * corr_vision[0];

			y_est[2] = y_est_prev[2] - beta2 * fal(corr_vision[1], 0.75, delta);
			y_est[1] = y_est_prev[1] + dt * (y_est_prev[2] + acc[1]) - beta1 * fal(corr_vision[1], 0.5, delta);
			y_est[0] = y_est_prev[0] + dt * y_est_prev[1] - beta0 * corr_vision[1];

		} else {
			
		}

		if (t > pub_last + PUB_INTERVAL) {
			pub_last = t;

			/* push current estimate to buffer */
			est_buf[buf_ptr][0][0] = x_est[0];
			est_buf[buf_ptr][0][1] = x_est[1];
			est_buf[buf_ptr][1][0] = y_est[0];
			est_buf[buf_ptr][1][1] = y_est[1];
			est_buf[buf_ptr][2][0] = z_est[0];
			est_buf[buf_ptr][2][1] = z_est[1];

			/* push current rotation matrix to buffer */
			memcpy(R_buf[buf_ptr], att.R, sizeof(att.R));

			buf_ptr++;

			if (buf_ptr >= EST_BUF_SIZE) {
				buf_ptr = 0;
			}

			for(int i = Average-1; i > 0; i--)
			{
				vel_buf[i][0] = vel_buf[i-1][0];
				vel_buf[i][1] = vel_buf[i-1][1];
			}
			vel_buf[0][0] = x_est[1];
			vel_buf[0][1] = y_est[1];
			float sum_x = 0.0f;
			float sum_y = 0.0f;
			for(int i = 0; i < Average; i ++)
			{
				sum_x = sum_x + vel_buf[i][0];
				sum_y = sum_y + vel_buf[i][1];
			}
			x_est[1] = sum_x/Average;
			y_est[1] = sum_y/Average;

			/* publish local position */
			test.local_x = x_est[0];
			test.local_vx = x_est[1];
			test.local_y = y_est[0];
			test.local_vy = y_est[1];

			x_est_prev[0] = x_est[0];
			x_est_prev[1] = x_est[1];
			x_est_prev[2] = x_est[2];
			y_est_prev[0] = y_est[0];
			y_est_prev[1] = y_est[1];
			y_est_prev[2] = y_est[2];


			orb_publish(ORB_ID(test), predict_position_pub, &test);
		}

		usleep(10000);
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[inav] stopped");
	thread_running = false;
	return 0;
}
