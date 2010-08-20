#define LOG_TAG "Sensors"

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>

#include <linux/input.h>
#include "bma150.h"

#include <cutils/log.h>
#include <cutils/atomic.h>

/*****************************************************************************/

#define BMA_DEVICE_NAME             "/dev/bma150"

#define SUPPORTED_SENSORS  (SENSOR_TYPE_ACCELEROMETER)

// sensor IDs must be a power of two and
// must match values in SensorManager.java
#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Z
#define EVENT_TYPE_ACCEL_Z          ABS_Y
#define EVENT_TYPE_ACCEL_STATUS     ABS_WHEEL

#define CONVERT_A                   (GRAVITY_EARTH / 256.0f)
#define CONVERT_A_X                 (CONVERT_A)
#define CONVERT_A_Y                 (CONVERT_A)
#define CONVERT_A_Z                 (CONVERT_A)

/*****************************************************************************/

static int sBmaFD = -1;
static uint32_t sActiveSensors = 0;

/*****************************************************************************/

/** State information for each device instance */
struct sensors_context_t {
    struct sensors_control_device_t device_control;
    struct sensors_data_device_t device_data;
};

/**
 * Common hardware methods
 */

static int open_sensors(const struct hw_module_t* module, const char* name, struct hw_device_t** device);

static struct hw_module_methods_t sensors_module_methods = {
	    .open =  open_sensors
};

/*
 * The Sensors Module
 */
int sensors_get_sensors(struct sensors_module_t* module, struct sensor_t const** sensors);

const struct sensors_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.version_major = 1,
		.version_minor = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "LGE SENSORS Module",
		.author = "LGE, Inc.",
		.methods = &sensors_module_methods,
	},
	.get_sensors_list = sensors_get_sensors
};

const struct sensor_t LGE_SENSOR = {
	.name = "accelerometer",
	.vendor = "bma150",
	.version = 1,
	.handle = 1,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.maxRange = 512.0f,
	.resolution = 512.0f,
	.power = 1.0f,
};

static int open_input()
{
    /* scan all input drivers and look for "accelerometer" */
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        if(de->d_name[0] == '.' &&
           (de->d_name[1] == '\0' ||
            (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        if (fd>=0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            if (!strcmp(name, "accelerometer")) {
                break;
            }
            close(fd);
            fd = -1;
        }
    }
    closedir(dir);

    if (fd < 0) {
        LOGE("Couldn't find or open 'accelerometer' driver (%s)", strerror(errno));
    }
    return fd;
}

static int open_bma()
{
    if (sBmaFD <= 0) {
        sBmaFD = open(BMA_DEVICE_NAME, O_RDONLY);
        LOGE_IF(sBmaFD<0, "Couldn't open %s (%s)",
                BMA_DEVICE_NAME, strerror(errno));
        if (sBmaFD >= 0) {
            sActiveSensors = 0;
        }
    }
    return sBmaFD;
}

static void close_bma()
{
    if (sBmaFD > 0) {
        close(sBmaFD);
        sBmaFD = -1;
    }
}

static void enable_disable(int fd, uint32_t sensors, uint32_t mask)
{
    if (fd<0) return;
    short flags;

    //LOGD("%s, fd=%d, sensors=%08x, mask=%08x", __PRETTY_FUNCTION__, fd, sensors, mask);
	if (mask & SENSOR_TYPE_ACCELEROMETER) {
        flags = (sensors & SENSOR_TYPE_ACCELEROMETER) ? 1 : 0;
		if (flags) {
			if (ioctl(fd, ACCEL_IOC_ENABLE)) {
				LOGE("ACCEL_IOC_ENABLE error (%s)", strerror(errno));
			} else {
				LOGD("ACCEL_IOC_ENABLE ACCELERATION OK!!");
			}
		} else {
			if (ioctl(fd, ACCEL_IOC_DISABLE)) {
				LOGE("ACCEL_IOC_DISABLE error (%s)", strerror(errno));
			} else {
				LOGD("ACCEL_IOC_DISABLE ACCELERATION OK!!");
			}
		}
	}
}

static uint32_t read_sensors_state(int fd)
{
    if (fd<0) return 0;
    uint32_t sensors = 0;
    return sensors;
}

/*****************************************************************************/

uint32_t sensor_control_init()
{
    return SUPPORTED_SENSORS;
}

int sensor_control_open(struct sensors_control_device_t *dev)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx)
	    return open_input();
	else
		return -1;
}

int sensor_control_activate(struct sensors_control_device_t *dev, int sensors, int mask)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) {
		mask &= SUPPORTED_SENSORS;
		uint32_t active = sActiveSensors;
		uint32_t new_sensors = (active & ~mask) | (sensors & mask);
		uint32_t changed = active ^ new_sensors;
		//LOGD("LGE: hopemini sensor_control_activate() sensors=%08x, mask=%08x, active=%08x, new_sensors= %08x, changed=%08x", sensors, mask, active, new_sensors, changed);
		if (changed) {
			int fd = open_bma();
			if (fd < 0) return -1;

			if (!active && new_sensors) {
				// force all sensors to be updated
				changed = SUPPORTED_SENSORS;
			}

			enable_disable(fd, new_sensors, changed);

			if (active && !new_sensors) {
				// close the driver
				close_bma();
			}
			sActiveSensors = active = new_sensors;
		}
		return 0;
	} else {
		return -1;
	}
}

int sensor_control_delay(struct sensors_control_device_t *dev, int32_t ms)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) {
		if (sBmaFD <= 0) {
			return -1;
		}
		short delay = ms;
		if (delay < 10)	delay = 10;
		if (ioctl(sBmaFD, ACCEL_IOCS_SAMPLERATE, delay) < 0) {
			LOGD("LGE: hopemini delay error!!");
			return -errno;
		}
		return 0;
	} else
		return -1;
}

int sensor_control_wake(struct sensors_control_device_t *dev)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) { 
		return -EWOULDBLOCK;
	} else {
		return -1;
	}
}

/*****************************************************************************/

#define MAX_NUM_SENSORS 1
static int sInputFD = -1;
static const int ID_A  = 0;
//static const int ID_A  = 1;
//static const int ID_OR = 7; // orientation raw
static sensors_data_t sSensors[MAX_NUM_SENSORS];
static uint32_t sPendingSensors;

int sensor_data_open(struct sensors_data_device_t *dev, int fd)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) {
		int i;
		memset(&sSensors, 0, sizeof(sSensors));
		for (i=0 ; i<MAX_NUM_SENSORS ; i++) {
			// by default all sensors have high accuracy
			// (we do this because we don't get an update if the value doesn't
			// change).
			sSensors[i].vector.status = SENSOR_STATUS_ACCURACY_HIGH;
		}
		sPendingSensors = 0;
		sInputFD = dup(fd);
		return 0;
	} else {
		return -1;
	}
}

int sensor_data_close(struct sensors_data_device_t *dev)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) {
		close(sInputFD);
		sInputFD = -1;
		return 0;
	} else {
		return -1;
	}
}

static int pick_sensor(sensors_data_t* values)
{
    uint32_t mask = 0x000000FF;
    while(mask) {
        uint32_t i = 31 - __builtin_clz(mask);
        mask &= ~(1<<i);
        if (sPendingSensors & (1<<i)) {
            sPendingSensors &= ~(1<<i);
            *values = sSensors[i];
            values->sensor = (1<<i);
            //LOGD("%d [%f, %f, %f]", (1<<i), values->vector.x, values->vector.y, values->vector.z);
            return (1<<i);
        }
    }
    LOGE("No sensor to return!!! sPendingSensors=%08x", sPendingSensors);
    // we may end-up in a busy loop, slow things down, just in case.
    usleep(100000);
    return -1;
}

float toDegreeX(int x) {
	float y = 180  - (float)(180 * x / 255);
	return y;
}

float toDegree(int x) {
	float y = (float)(-1 * 90 * x / 255);
	return y;
}

int sensor_data_poll(struct sensors_data_device_t *dev, sensors_data_t* values)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) {
		struct input_event event;
		int nread;
		int64_t t;

		int fd = sInputFD;
		if (fd <= 0)
			return -1;

		// there are pending sensors, returns them now...
		if (sPendingSensors) {
			//LOGD("sensors_data_poll: pick_sensor(values=%d)", values);
			return pick_sensor(values);
		}

		uint32_t new_sensors = 0;
		struct pollfd fds;
		fds.fd = fd;
		fds.events = POLLIN;
		fds.revents = 0;

		// wait until we get a complete event for an enabled sensor
		while (1) {
			nread = 0;
			if (nread == 0) {
				/* read the next event */
				nread = read(fd, &event, sizeof(event));
			}
			if (nread == sizeof(event)) {
				uint32_t v;
				if (event.type == EV_ABS) {
					switch (event.code) {

						case EVENT_TYPE_ACCEL_X:
							new_sensors |= SENSOR_TYPE_ACCELEROMETER;
							sSensors[ID_A].acceleration.x = (float)event.value * CONVERT_A_X;
							break;
						case EVENT_TYPE_ACCEL_Y:
							new_sensors |= SENSOR_TYPE_ACCELEROMETER;
							sSensors[ID_A].acceleration.y = (float)event.value * CONVERT_A_Y;
							break;
						case EVENT_TYPE_ACCEL_Z:
							new_sensors |= SENSOR_TYPE_ACCELEROMETER;
							sSensors[ID_A].acceleration.z = (float)event.value * CONVERT_A_Z;
							break;
					}
				} else if (event.type == EV_SYN) {
					if (new_sensors) {
						sPendingSensors = new_sensors;
						int64_t t = event.time.tv_sec*1000000000LL +
								event.time.tv_usec*1000;
						while (new_sensors) {
							uint32_t i = 31 - __builtin_clz(new_sensors);
							new_sensors &= ~(1<<i);
							sSensors[i].time = t;
						}
						return pick_sensor(values);
					}
				} else {
					LOGD("hopemini :::: NO TYPE ::: type: %d code: %d value: %-5d time: %ds", event.type, event.code, event.value, (int)event.time.tv_sec);
				}
			}
		}
	} else {
		return -1;
	}
}

int sensors_get_sensors(struct sensors_module_t* module, struct sensor_t const** sensors) {

	*sensors = &LGE_SENSOR;
    return 1;
}

static int close_sensors(struct hw_device_t *dev) 
{
	struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	if (ctx) {
		free(ctx);
	}
	return 0;
}

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* name, struct hw_device_t** device)
{
	struct sensors_context_t *ctx = malloc(sizeof(struct sensors_context_t));
	memset(ctx, 0, sizeof(*ctx));

    ctx->device_data.common.tag = HARDWARE_DEVICE_TAG;
    ctx->device_data.common.version = 0;
    ctx->device_data.common.module = module;
    ctx->device_data.common.close = close_sensors;
	if (!strcmp(name, SENSORS_HARDWARE_DATA)) {
		ctx->device_data.data_open = sensor_data_open;
		ctx->device_data.data_close = sensor_data_close;
		ctx->device_data.poll = sensor_data_poll;
		*device = &ctx->device_data.common;
	} else if (!strcmp(name, SENSORS_HARDWARE_CONTROL)) {
		ctx->device_control.open_data_source = sensor_control_open;
		ctx->device_control.activate = sensor_control_activate;
		ctx->device_control.set_delay = sensor_control_delay;
		ctx->device_control.wake = sensor_control_wake;
		*device = &ctx->device_control.common;
	}

	return 0;
}

