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
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/gps_position.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>

#include "inertial_filter.h"

#define MIN_VALID_W 0.00001f
#define PUB_INTERVAL 10000	// limit publish rate to 100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL		// buffer size is 0.5s
#define Delay_gps 0.2f
#define W_xy_gps_p 1.0f
#define W_acc_bias 0.05f
#define W_xy_res_v 0.5f

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int inertial_filter_gps_task; /**< Handle of deamon task / thread */

static const hrt_abstime gps_topic_timeout = 500000;		// GPS topic timeout = 0.5s

__EXPORT int inertial_filter_gps_main(int argc, char *argv[]);
int inertial_filter_gps_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
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

	fprintf(stderr, "usage: inertial_filter_gps {start|stop|status} [-v]\n\n");
	exit(1);
}

/**
 * The position_estimator_inav_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int inertial_filter_gps_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		inertial_filter_gps_task = task_spawn_cmd("inertial_filter_gps",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 5000,
					       inertial_filter_gps_thread_main,
					       (argv) ? (char * const *) &argv[2] : (char * const *) NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/****************************************************************************
 * main
 ****************************************************************************/
int inertial_filter_gps_thread_main(int argc, char *argv[])
{
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	float x_est[2] = { 0.0f, 0.0f };	// pos, vel
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel
	float z_est[2] = { 0.0f, 0.0f };	// pos, vel

	float est_buf[EST_BUF_SIZE][3][2];	// estimated position buffer
	float R_buf[EST_BUF_SIZE][3][3];	// rotation matrix buffer
	float R_gps[3][3];					// rotation matrix for GPS correction moment
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	int buf_ptr = 0;

	static const float min_eph_epv = 2.0f;	// min EPH/EPV, used for weight calculation
	static const float max_eph_epv = 20.0f;	// max EPH/EPV acceptable for estimation

	float eph = max_eph_epv;
	float epv = 1.0f;

	float x_est_prev[2], y_est_prev[2], z_est_prev[2];
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));

	bool ref_inited = false;
	hrt_abstime ref_init_start = 0;
	const hrt_abstime ref_init_delay = 1000000;	// wait for 1s after 3D fix
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));

	hrt_abstime accel_timestamp = 0;

	hrt_abstime pub_last = hrt_absolute_time();
	hrt_abstime t_prev = 0;

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
	float corr_gps[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
	float w_gps_xy = 1.0f;
	bool gps_valid = false;			// GPS is valid

	/* declare and safely initialize all structs */
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct gps_position_s gps_pos;
	memset(&gps_pos, 0, sizeof(gps_pos));

	/* subscribe */
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	/* advertise */
	orb_advert_t gps_position_pub = orb_advertise(ORB_ID(gps_position), &gps_pos);

	thread_running = true;
	/* main loop */
	struct pollfd fds[1] = {
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
	};

	while (!thread_should_exit) {
		int ret = poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* act on attitude updates */

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

			/* vehicle GPS position */
			orb_check(vehicle_gps_position_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				bool reset_est = false;

				/* hysteresis for GPS quality */
				if (gps_valid) {
					if (gps.eph > max_eph_epv || gps.epv > max_eph_epv || gps.fix_type < 3) {
						gps_valid = false;
						mavlink_log_info(mavlink_fd, "[inav] GPS signal lost");
					}

				} else {
					if (gps.eph < max_eph_epv * 0.7f && gps.epv < max_eph_epv * 0.7f && gps.fix_type >= 3) {
						gps_valid = true;
						reset_est = true;
						mavlink_log_info(mavlink_fd, "[inav] GPS signal found");
					}
				}

				if (gps_valid) {
					double lat = gps.lat * 1e-7;
					double lon = gps.lon * 1e-7;
					float alt = gps.alt * 1e-3;

					/* initialize reference position if needed */
					if (!ref_inited) {
						if (ref_init_start == 0) {
							ref_init_start = t;

						} else if (t > ref_init_start + ref_init_delay) {
							ref_inited = true;

							/* set position estimate to (0, 0, 0), use GPS velocity for XY */
							x_est[0] = 0.0f;
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = 0.0f;
							y_est[1] = gps.vel_e_m_s;

							/* initialize projection */
							map_projection_init(&ref, lat, lon);
							// XXX replace this print
							warnx("init ref: lat=%.7f, lon=%.7f, alt=%8.4f", (double)lat, (double)lon, (double)alt);
							mavlink_log_info(mavlink_fd, "[inav] init ref: %.7f, %.7f, %8.4f", (double)lat, (double)lon, (double)alt);
						}
					}

					if (ref_inited) {
						/* project GPS lat lon to plane */
						float gps_proj[2];
						map_projection_project(&ref, lat, lon, &(gps_proj[0]), &(gps_proj[1]));

						/* reset position estimate when GPS becomes good */
						if (reset_est) {
							x_est[0] = gps_proj[0];
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = gps_proj[1];
							y_est[1] = gps.vel_e_m_s;
						}

						/* calculate index of estimated values in buffer */
						int est_i = buf_ptr - 1 - min(EST_BUF_SIZE - 1, max(0, (int)(Delay_gps * 1000000.0f / PUB_INTERVAL)));
						if (est_i < 0) {
							est_i += EST_BUF_SIZE;
						}

						/* calculate correction for position */
						corr_gps[0][0] = gps_proj[0] - est_buf[est_i][0][0];
						corr_gps[1][0] = gps_proj[1] - est_buf[est_i][1][0];

						/* calculate correction for velocity */
						if (gps.vel_ned_valid) {
							corr_gps[0][1] = gps.vel_n_m_s - est_buf[est_i][0][1];
							corr_gps[1][1] = gps.vel_e_m_s - est_buf[est_i][1][1];
						} else {
							corr_gps[0][1] = 0.0f;
							corr_gps[1][1] = 0.0f;
						}

						/* save rotation matrix at this moment */
						memcpy(R_gps, R_buf[est_i], sizeof(R_gps));

						w_gps_xy = min_eph_epv / fmaxf(min_eph_epv, gps.eph);
					}

				} else {
					/* no GPS lock */
					memset(corr_gps, 0, sizeof(corr_gps));
					ref_init_start = 0;
				}
			}
		}


		/* check for timeout on GPS topic */
		if (gps_valid && (t > (gps.timestamp_position + gps_topic_timeout))) {
			gps_valid = false;
			warnx("GPS timeout");
			mavlink_log_info(mavlink_fd, "[inav] GPS timeout");
		}

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.002);		// constrain dt from 2 to 20 ms
		t_prev = t;

		/* increase EPH/EPV on each step */
		if (eph < max_eph_epv) {
			eph *= 1.0f + dt;
		}
		if (epv < max_eph_epv) {
			epv += 0.005f * dt;	// add 1m to EPV each 200s (baro drift)
		}

		/* use GPS if it's valid and reference position initialized */
		bool use_gps_xy = ref_inited && gps_valid ;
		bool can_estimate_xy = (eph < max_eph_epv) || use_gps_xy;

		float w_xy_gps_p = W_xy_gps_p * w_gps_xy;
		float w_xy_gps_v = W_xy_gps_p * w_gps_xy;

		/* accelerometer bias correction for GPS (use buffered rotation matrix) */
		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };

		if (use_gps_xy) {
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
		}

		/* transform error vector from NED frame to body frame */
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R_gps[j][i] * accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * W_acc_bias * dt;
			}
		}

		if (can_estimate_xy) {
			/* inertial filter prediction for position */
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);

			/* inertial filter correction for position */
			if (use_gps_xy) {
				eph = fminf(eph, gps.eph);

				inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
				inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

				if (gps.vel_ned_valid && t < gps.timestamp_velocity + gps_topic_timeout) {
					inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
					inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
				}
			}

			
		} else {
			/* gradually reset xy velocity estimates */
			inertial_filter_correct(-x_est[1], dt, x_est, 1, W_xy_res_v);
			inertial_filter_correct(-y_est[1], dt, y_est, 1, W_xy_res_v);
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

			gps_pos.timestamp = t;
			gps_pos.x = x_est[0];
			gps_pos.vx = x_est[1];
			gps_pos.y = y_est[0];
			gps_pos.vy = y_est[1];

			orb_publish(ORB_ID(gps_position), gps_position_pub, &gps_pos);
		}

		usleep(10000);
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[inav] stopped");
	thread_running = false;
	return 0;
}
