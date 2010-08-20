/*
 * Copyright 2008, Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Sensors"

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>

#include <linux/input.h>
//#include "bma150.h"

#include <cutils/log.h>
#include <cutils/atomic.h>

/*****************************************************************************/
#define SENSOR_TYPE_PROXIMITY 		8 //sensors.h
#define PROXI_DEVICE_NAME             "/dev/proximity" /*MISC_DEV_NAME : Proximity Sensors*/
#define SUPPORTED_PSENSORS    SENSOR_TYPE_PROXIMITY //1//| SENSOR_TYPE_ACCELEROMETER )

// sensor IDs must be a power of two and
// must match values in SensorManager.java
#define EVENT_ABS_DISTANCE          0x19  /*ABS_DISTANCE*/
#define EVENT_TYPE_PROXIMITY        EVENT_ABS_DISTANCE
//#define EVENT_TYPE_ACCEL_Z          ABS_Y
//#define EVENT_TYPE_ACCEL_STATUS     ABS_WHEEL

#define PSENSOR_STATE_MASK           (0x7FFF)

/*****************************************************************************/

static int sProxiFD = -1; //sBmaFD
static uint32_t sActivePsensors = 0;
static uint32_t Psensor; 

/*****************************************************************************/
/* file : bma150.h */

#define PROXI_IO_TYPE	'B' /*?*/

#define PROXI_IOC_ENABLE	_IO(PROXI_IO_TYPE, 1)
#define PROXI_IOC_DISABLE	_IO(PROXI_IO_TYPE, 2)
#define PROXI_IOCS_SAMPLERATE	_IOW(PROXI_IO_TYPE, 3, int) /* in ms */
#define PROXI_IOCG_SAMPLERATE	_IO(PROXI_IO_TYPE, 4) /* in ms */


/*****************************************************************************/
/** State information for each device instance */
struct psensors_context_t {
    struct sensors_control_device_t device_control;
    struct sensors_data_device_t device_data;
};


/**
 * Common hardware methods
 */

static int open_psensors(const struct hw_module_t* module, const char* name, struct hw_device_t** device);

static struct hw_module_methods_t psensors_module_methods = {
	    .open =  open_psensors
};


/*
 * The Sensors Module
 */
int psensors_get_sensors(struct sensors_module_t* module, struct sensor_t const** sensors);


const struct sensors_module_t HAL_MODULE_INFO_SYM_2 = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.version_major = 1,
		.version_minor = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		//.id = PSENSORS_HARDWARE_MODULE_ID,
		.name = "LGE SENSORS Module",
		.author = "LGE, Inc.",
		.methods = &psensors_module_methods,
	},
	.get_sensors_list = psensors_get_sensors
};


#if 0
const struct sensor_t LGE_PROXIMITY_SENSOR = {
	.name = "accelerometer",
	.vendor = "bma150",
	.version = 1,
	.handle = 1,
	.type = SENSOR_TYPE_RPOXIMITY, /*SENSOR_TYPE_ACCELEROMETER,*/
	.maxRange = 512.0f,
	.resolution = 512.0f,
	.power = 1.0f,
};
#else
const struct sensor_t LGE_PROXIMITY_SENSOR = {
	.name = "proximity", //"accelerometer",
	.vendor = "proxi-sensor", //"gp2ap002",
	.version = 1,
	.handle = 1,
//	.type = SENSOR_TYPE_ORIENTATION,
//	.type = SENSOR_TYPE_ACCELEROMETER,
	.type = SENSOR_TYPE_PROXIMITY,
	.maxRange = 1.0f,
	.resolution = 1.0f,
	.power = 1.0f,
};
#endif
/*
 * We use a Least Mean Squares filter to smooth out the output of the yaw
 * sensor.
 *
 * The goal is to estimate the output of the sensor based on previous acquired
 * samples.
 *
 * We approximate the input by a line with the equation:
 *      Z(t) = a * t + b
 *
 * We use the Least Mean Squares method to calculate a and b so that the
 * distance between the line and the measured COUNT inputs Z(t) is minimal.
 *
 * In practice we only need to compute b, which is the value we're looking for
 * (it's the estimated Z at t=0). However, to improve the latency a little bit,
 * we're going to discard a certain number of samples that are too far from
 * the estimated line and compute b again with the new (trimmed down) samples.
 *
 * notes:
 * 'a' is the slope of the line, and physicaly represent how fast the input
 * is changing. In our case, how fast the yaw is changing, that is, how fast the
 * user is spinning the device (in degre / nanosecond). This value should be
 * zero when the device is not moving.
 *
 * The minimum distance between the line and the samples (which we are not
 * explicitely computing here), is an indication of how bad the samples are
 * and gives an idea of the "quality" of the estimation (well, really of the
 * sensor values).
 *
 */

#if 0
/* sensor rate in me */
#define SENSORS_RATE_MS     500    /*20*/
/* timeout (constant value) in ms */
#define SENSORS_TIMEOUT_MS  1000   /*100*/
/* # of samples to look at in the past for filtering */
#define COUNT               24
/* prediction ratio */
#define PREDICTION_RATIO    (1.0f/3.0f)
/* prediction time in seconds (>=0) */
#define PREDICTION_TIME     ((SENSORS_RATE_MS*COUNT/1000.0f)*PREDICTION_RATIO)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#if 0
#endif

/*****************************************************************************/
#if 1
static int open_input_psensor()
{
    /* scan all input drivers and look for "accelerometer" */
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);

	LOGD("diyu type: open_input_psensor\n");
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
            if (!strcmp(name, "proximity")) {  //input_dev->name :
                LOGD("using %s (name=%s)", devname, name);
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

static int open_proxi()
{
	LOGD("diyu type: open_proxi\n");

    if (sProxiFD <= 0) {
        sProxiFD = open(PROXI_DEVICE_NAME, O_RDONLY);
        LOGE_IF(sProxiFD<0, "Couldn't open %s (%s)",
                PROXI_DEVICE_NAME, strerror(errno));
        if (sProxiFD >= 0) {
            sActivePsensors = 0;
        }
    }
    return sProxiFD;
}

static void close_proxi()
{
	LOGD("diyu type: close_proxi\n");

    if (sProxiFD > 0) {
        LOGD("%s, fd=%d", __PRETTY_FUNCTION__, sProxiFD);
        close(sProxiFD);
        sProxiFD = -1;
    }
}

static void enable_disable_psensor(int fd, uint32_t sensors, uint32_t mask)
{
    if (fd<0) return;
    short flags;
    Psensor = sensors;

	LOGD("diyu type: enable_disable_psensor\n");

    if (sensors & SENSOR_TYPE_PROXIMITY) {
        mask |= SENSOR_TYPE_PROXIMITY;
    }

  
    if (mask & SENSOR_TYPE_PROXIMITY) {
        flags = (sensors & SENSOR_TYPE_PROXIMITY) ? 1 : 0;
		
		if (flags) {
			if (ioctl(fd, PROXI_IOC_ENABLE)) {
				LOGE("PROXI_IOC_ENABLE error Orientation (%s)", strerror(errno));
			} else {
				LOGD("PROXI_IOC_ENABLE ACCELERATION OK!!");
			}
		} else {
			if (ioctl(fd, PROXI_IOC_DISABLE)) {
				LOGE("PROXI_IOC_DISABLE error (%s)", strerror(errno));
			} else {
				LOGD("PROXI_IOC_DISABLE ACCELERATION OK!!");
			}
		}
        /*if (ioctl(fd, ECS_IOCTL_APP_SET_MFLAG, &flags) < 0) {
            LOGE("ECS_IOCTL_APP_SET_MFLAG error (%s)", strerror(errno));
        }*/
    }
    if (mask & SENSOR_TYPE_PROXIMITY) {
        flags = (sensors & SENSOR_TYPE_PROXIMITY) ? 1 : 0;

		if (flags) {
			if (ioctl(fd, PROXI_IOC_ENABLE)) {
				LOGE("PROXI_IOC_ENABLE error Accelerometer (%s)", strerror(errno));
			} else {
				LOGD("PROXI_IOC_ENABLE ACCELERATION OK!!");
			}
		} else {
			if (ioctl(fd, PROXI_IOC_DISABLE)) {
				LOGE("PROXI_IOC_DISABLE error (%s)", strerror(errno));
			} else {
				LOGD("PROXI_IOC_DISABLE ACCELERATION OK!!");
			}

		}
        /*if (ioctl(fd, ECS_IOCTL_APP_SET_AFLAG, &flags) < 0) {
            LOGE("ECS_IOCTL_APP_SET_AFLAG error (%s)", strerror(errno));
        }*/
    }
}

static uint32_t read_sensors_state_psensor(int fd)
{
    if (fd<0) return 0;
    short flags;
    uint32_t sensors = 0;

	LOGD("diyu type: read_sensors_state_psensor\n");
    // read the actual value of all sensors
/*    if (!ioctl(fd, ECS_IOCTL_APP_GET_MFLAG, &flags)) {
        if (flags)  sensors |= SENSOR_TYPE_ORIENTATION;
        else        sensors &= ~SENSOR_TYPE_ORIENTATION;
    }
    if (!ioctl(fd, ECS_IOCTL_APP_GET_AFLAG, &flags)) {
        if (flags)  sensors |= SENSOR_TYPE_ACCELEROMETER;
        else        sensors &= ~SENSOR_TYPE_ACCELEROMETER;
    }*/
    
    return sensors;
}

/*****************************************************************************/

uint32_t psensor_control_init()
{
    return SUPPORTED_PSENSORS;
}

int psensor_control_open(struct sensors_control_device_t *dev)
{
    struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
	
	 LOGD("diyu type: psensor_control_open\n");
	if (ctx)
    return open_input_psensor();
	else
		return -1;
}

int psensor_control_activate(struct sensors_control_device_t *dev, int sensors, int mask)
{
    struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
    if (ctx) {
         mask &= SUPPORTED_PSENSORS;
         uint32_t active = sActivePsensors;
         uint32_t new_sensors = (active & ~mask) | (sensors & mask);
         uint32_t changed = active ^ new_sensors;
    //LOGD("LGE: hopemini sensor_control_activate() sensors=%08x, mask=%08x, active=%08x, new_sensors= %08x, changed=%08x", sensors, mask, active, new_sensors, changed);
    
	LOGD("diyu type: psensor_control_activate\n");

        if (changed) {
             int fd = open_proxi();
			if (fd < 0) return -1;

             if (!active && new_sensors) {
            // force all sensors to be updated
                 changed = SUPPORTED_PSENSORS;
             }

             enable_disable_psensor(fd, new_sensors, changed);

             if (active && !new_sensors) {
            // close the driver
                close_proxi();
             }
             sActivePsensors = active = new_sensors;
            LOGD("sensors=%08x, real=%08x",
                sActivePsensors, read_sensors_state_psensor(fd));
             }
     	    return 0;
       } else {
 	    return -1;
       }
}

int psensor_control_delay(struct sensors_control_device_t *dev, int32_t ms)
{
    struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
	LOGD("diyu type: psensor_control_delay\n");
    if (ctx) {
        if (sProxiFD <= 0) {
            return -1;
        }
        short delay = ms;
		if (delay < 10)	delay = 10;
		if (ioctl(sProxiFD, PROXI_IOCS_SAMPLERATE, delay) < 0) {
			LOGD("LGE: hopemini delay error!!");
        return -errno;
        }
       return 0;
   } else
       return -1;
}

int psensor_control_wake(struct sensors_control_device_t *dev)
{
    struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
	LOGD("diyu type: psensor_control_wake\n");
	if (ctx) { 
		return -EWOULDBLOCK;
	} else {
		return -1;
	}
}

/*****************************************************************************/

#define MAX_NUM_PSENSORS 1
static int pInputFD = -1;
//static const int ID_O  = 0;//orientation
//static const int ID_A  = 1;//original : accelerometer
static const int ID_P  = 5;//proximity //diyu@lge.com //2^5=32
static sensors_data_t pSensors[MAX_NUM_PSENSORS];
static uint32_t pPendingSensors;

int psensor_data_open(struct sensors_data_device_t *dev, int fd)
{
   struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
   if (ctx) {
     int i;
    memset(&pSensors, 0, sizeof(pSensors));
		LOGD("diyu type: psensor_data_open\n");
    for (i=0 ; i<MAX_NUM_PSENSORS ; i++) {
        // by default all sensors have high accuracy
        // (we do this because we don't get an update if the value doesn't
        // change).
        pSensors[i].vector.status = SENSOR_STATUS_ACCURACY_HIGH;
    }
    pPendingSensors = 0;
    pInputFD = dup(fd);
    LOGD("psensors_data_open: fd = %d", pInputFD);
        return 0;
    } else {
	return -1;
    }
}

int psensor_data_close(struct sensors_data_device_t *dev)
{
    struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
	LOGD("diyu type: psensor_data_close\n");
	if (ctx) {
    close(pInputFD);
    pInputFD = -1;
    return 0;
	} else {
		return -1;
	}
}

static int pick_psensor(sensors_data_t* values)
{
    uint32_t mask = 0x000000FF;
    while(mask) {
		LOGD("diyu type: pick_psensor\n");
        uint32_t i = 31 - __builtin_clz(mask);
        mask &= ~(1<<i);
        if (pPendingSensors & (1<<i)) {
            pPendingSensors &= ~(1<<i);
            *values = pSensors[i];
            values->sensor = (1<<i);
			LOGD("diyu %d [%f, %f, %f]", (1<<i),
                    values->vector.x,
                    values->vector.y,
                    values->vector.z);
            
            LOGD_IF(0, "%d [%f, %f, %f]", (1<<i),
                    values->vector.x,
                    values->vector.y,
                    values->vector.z);
            return (1<<i);
        }
    }
    LOGE("No sensor to return!!! pPendingSensors=%08x", pPendingSensors);
    // we may end-up in a busy loop, slow things down, just in case.
    usleep(100000);
    return -1;
}


//static int test_value = 90;

int psensor_data_poll(struct sensors_data_device_t *dev, sensors_data_t* values)
{
    struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
	if (ctx) {
	    struct input_event event;
    	int nread;
    	int64_t t;

	    int fd = pInputFD;
	    if (fd <= 0)
	        return -1;

		LOGD("diyu type: psensor_data_poll\n");

	    // there are pending sensors, returns them now...
	    if (pPendingSensors) {
			LOGD("diyu type: pPendingSensors\n");
	        return pick_psensor(values);
	    }

	    uint32_t new_sensors = 0;
	    struct pollfd fds;
	    fds.fd = fd;
	    fds.events = POLLIN;
	    fds.revents = 0;

	    // wait until we get a complete event for an enabled sensor
	    while (1) {
	        nread = 0;
			#if 0
	        if (mSensor & SENSOR_TYPE_ORIENTATION) {
	            /* We do some special processing if the orientation sensor is
	             * activated. In particular the yaw value is filtered with a
	             * LMS filter. Since the kernel only sends an event when the
	             * value changes, we need to wake up at regular intervals to
	             * generate an output value (the output value may not be
	             * constant when the input value is constant)
	             */
	            int err = poll(&fds, 1, SENSORS_TIMEOUT_MS);
	            if (err == 0) {
	                struct timespec time;
	                time.tv_sec = time.tv_nsec = 0;
	                clock_gettime(CLOCK_MONOTONIC, &time);
					LOGD("diyu 1000000000LL\n");
						
	                /* generate an output value */
	                t = time.tv_sec*1000000000LL+time.tv_nsec;
	                new_sensors |= SENSOR_TYPE_ORIENTATION;
					
	                /*sSensors[ID_O].orientation.roll=
	                        LMSFilter(t, sSensors[ID_O].orientation.roll);*/
					//per 180 degree 
					sSensors[ID_O].orientation.azimuth =
	                        LMSFilter(t, sSensors[ID_O].orientation.azimuth);
/*					sSensors[ID_O].orientation.yaw =
							LMSFilter(t, sSensors[ID_O].orientation.yaw);*/


	                /* generate a fake sensors event */
	                event.type = EV_SYN;
	                event.time.tv_sec = time.tv_sec;
	                event.time.tv_usec = time.tv_nsec/1000;
	                nread = sizeof(event);
	            }
	        }
			#endif
	        if (nread == 0) {
	            /* read the next event */
	            nread = read(fd, &event, sizeof(event));
	        }
	        if (nread == sizeof(event)) {
	            uint32_t v;
			if (event.type == EV_ABS) {
	                LOGD("diyu psensor: %d code: %d value: %-5d ",
	                        event.type, event.code, event.value);
	                switch (event.code) {
	                    case EVENT_TYPE_PROXIMITY:
	                        new_sensors |= SENSOR_TYPE_PROXIMITY;
							pSensors[ID_P].proximity.distance = (float)event.value ;
							LOGD("diyu type: PROXIMITY X %d\n", event.value);
	                        break;
					}
	            	
	            } else if (event.type == EV_SYN) {
					LOGD("diyu psensor: event.type == EV_SYN\n");
	                if (new_sensors) {
	                    pPendingSensors = new_sensors;
	                    int64_t t = event.time.tv_sec*1000000000LL +
	                            event.time.tv_usec*1000;
						//LOGD("diyu type:  if (new_sensors) {\n");
	                    while (new_sensors) {
	                        uint32_t i = 31 - __builtin_clz(new_sensors);
	                        new_sensors &= ~(1<<i);
	                        pSensors[i].time = t;
	                    }
	                    return pick_psensor(values);
	                }
	            }
	        }
	    }

	}
}

int psensors_get_sensors(struct sensors_module_t* module, struct sensor_t const** sensors) {
	*sensors = &LGE_PROXIMITY_SENSOR;
//    return SUPPORTED_PSENSORS;
	LOGD("diyu type: psensors_get_sensors\n");

    return 1;
}

static int close_psensors(struct hw_device_t *dev) 
{
	struct psensors_context_t* ctx = (struct psensors_context_t*)dev;
	LOGD("diyu type: close_psensors\n");
	if (ctx) {
		free(ctx);
	}
	return 0;
}

/** Open a new instance of a sensor device using name */
static int open_psensors(const struct hw_module_t* module, const char* name, struct hw_device_t** device)
{
	struct psensors_context_t *ctx = malloc(sizeof(struct psensors_context_t));
	memset(ctx, 0, sizeof(*ctx));
	LOGD("diyu type: open_psensors\n");

    ctx->device_data.common.tag = HARDWARE_DEVICE_TAG;
    ctx->device_data.common.version = 0;
    ctx->device_data.common.module = module;
    ctx->device_data.common.close = close_psensors;
	if (!strcmp(name, SENSORS_HARDWARE_DATA)) {
		ctx->device_data.data_open = psensor_data_open;
		ctx->device_data.data_close = psensor_data_close;
		ctx->device_data.poll = psensor_data_poll;
		*device = &ctx->device_data.common;
	LOGD("diyu type: open_psensors-1\n");
	} else if (!strcmp(name, SENSORS_HARDWARE_CONTROL)) {
		ctx->device_control.open_data_source = psensor_control_open;
		ctx->device_control.activate = psensor_control_activate;
		ctx->device_control.set_delay = psensor_control_delay;
		ctx->device_control.wake = psensor_control_wake;
		*device = &ctx->device_control.common;
	LOGD("diyu type: open_psensors-2\n");
	}

	return 0;
}
#endif
