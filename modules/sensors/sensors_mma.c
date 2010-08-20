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
#include "bma150.h"

#include <cutils/log.h>
#include <cutils/atomic.h>

/*****************************************************************************/

#define BMA_DEVICE_NAME             "/dev/bma150"

#define SUPPORTED_SENSORS  (SENSOR_TYPE_ORIENTATION  )//| SENSOR_TYPE_ACCELEROMETER )
//#define SUPPORTED_SENSORS  (SENSOR_TYPE_ORIENTATION  | SENSOR_TYPE_ACCELEROMETER )


// sensor IDs must be a power of two and
// must match values in SensorManager.java
#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Z
#define EVENT_TYPE_ACCEL_Z          ABS_Y
#define EVENT_TYPE_ACCEL_STATUS     ABS_WHEEL


#define EVENT_TYPE_YAW              ABS_RX/*ABS_RZ*//*ABS_RZ*//*ABS_RY*//*ABS_RY*//*ABS_RZ*//*ABS_RX*/
#define EVENT_TYPE_PITCH            ABS_RY/*ABS_RY*//*ABS_RZ*//*ABS_RZ*//*ABS_RX*//*ABS_RX*//*ABS_RY*/
#define EVENT_TYPE_ROLL             ABS_RZ/*ABS_RX*//*ABS_RX*//*ABS_RX*//*ABS_RZ*//*ABS_RY*//*ABS_RZ*/
#define EVENT_TYPE_ORIENT_STATUS    ABS_RUDDER

#define EVENT_TYPE_MAGV_X           ABS_HAT0X
#define EVENT_TYPE_MAGV_Y           ABS_HAT0Y
#define EVENT_TYPE_MAGV_Z           ABS_BRAKE

#define EVENT_TYPE_TEMPERATURE      ABS_THROTTLE
#define EVENT_TYPE_STEP_COUNT       ABS_GAS

// 720 LSG = 1G
#define LSG                         (32.0f) /*(720.0f)*/

// conversion of acceleration data to SI units (m/s^2)
#define CONVERT_A                   (GRAVITY_EARTH / LSG)
#define CONVERT_A_X                 (CONVERT_A)
#define CONVERT_A_Y                 (-CONVERT_A)
#define CONVERT_A_Z                 (CONVERT_A)

#define SENSOR_STATE_MASK           (0x7FFF)

/*****************************************************************************/

static int sBmaFD = -1;
static uint32_t sActiveSensors = 0;
static uint32_t mSensor; 


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

#if 0
static int  multivalue_degree[] = {
0,1,2,3,4,5,6,7,8,9,10,
11,12,13,14,15,16,17,18,19,20,
21,22,23,24,25,26,27,28,29,30,
31,
-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,
-11,-12,-13,-14,-15,-16,-17,-18,-19,-20,
-21,-22,-23,-24,-25,-26,-27,-28,-29,-30,
-31,-32
};

//	ARRAY_SIZE(multivalue_degree)
static float  multivalue_degree_xy_pair[64] = {
	0.0,
	2.69 , 5.38  ,8.08  , 10.81 , 13.55,
	16.33 , 19.16 , 22.02 , 24.95 , 27.95,
	31.04 , 34.23 , 37.54 , 41.01 , 44.68,
	48.59 , 52.83 , 57.54 , 62.95 , 69.64,
	79.86 , 10001 , 10002 , 10003, 10004 , 
	10005 , 10006 ,
	/*90.00 , 90.10 , 90.20 , 90.30,
	90.40 , 90.50 ,*/ 10007 , 10008 , 10009,
	10010 , 10011 , 10012 , 100013 , 100014,
	10015 , -90.50 , -90.40 , -90.30 , -90.20,
	-90.10 , -90.00 , -79.86, -69.64, -62.95,
	-57.54, -52.83, -48.59, -44.68, -41.01,
	-37.54, -34.23, -31.04, -27.95, -24.95,
	-22.02, -19.16, -16.33, -13.55, -10.81,
	-8.08, -5.38 , -2.69
};


//ARRAY_SIZE(multivalue_degree)
static float  multivalue_degree_z_pair[64] = {
  90.0,
  87.31 , 84.62 ,  81.92,  79.19,  76.45,
  73.67 , 70.84 ,  67.98,  65.05,  62.05,
  58.96 , 55.77 ,  52.46,  48.99,  45.32,
  41.41 , 37.17 ,  32.46,  27.05,  20.36,
  10.14 ,  
  10001 ,  10002,  10003,  10004 , 10005 , 
  10006 ,
  /*80.0 ,   0.0 ,   0.0,    0.0,
  0.0 , 0.0 ,*/ 10007 ,  10008,  10009,
  10010 , 10011 , 10012 ,  10013,  10014,
  10015 , 0.0 ,0.0 ,  0.0,  0.0,
  0.0 , 0.0 , -10.14, -20.36, -27.05,
 -32.46 , -37.17, -41.41, -45.36, -48.99,
 -52.46 , -55.77, -58.96, -62.05, -65.05,
 -67.98 , -70.84, -73.67, -76.45, -79.19,
 -81.92, -84.62 , -87.31
};
#endif


static float mV[COUNT*2];
static float mT[COUNT*2];
static int mIndex;

static inline
float normalize(float x)
{
    x *= (1.0f / 360.0f);
    if (fabsf(x) >= 0.5f)
        x = x - ceilf(x + 0.5f) + 1.0f;
    if (x < 0)
        x += 1.0f;
    x *= 360.0f;
    return x;
}

static void LMSInit(void)
{
    memset(mV, 0, sizeof(mV));
    memset(mT, 0, sizeof(mT));
    mIndex = COUNT;
}

static float LMSFilter(int64_t time, int v)
{
    const float ns = 1.0f / 1000000000.0f;
    const float t = time*ns;
    float v1 = mV[mIndex];
    if ((v-v1) > 180) {
        v -= 360;
    } else if ((v1-v) > 180) {
        v += 360;
    }
    /* Manage the circular buffer, we write the data twice spaced by COUNT
     * values, so that we don't have to memcpy() the array when it's full */
    mIndex++;
    if (mIndex >= COUNT*2)
        mIndex = COUNT;
    mV[mIndex] = v;
    mT[mIndex] = t;
    mV[mIndex-COUNT] = v;
    mT[mIndex-COUNT] = t;

    float A, B, C, D, E;
    float a, b;
    int i;

    A = B = C = D = E = 0;
    for (i=0 ; i<COUNT-1 ; i++) {
        const int j = mIndex - 1 - i;
        const float Z = mV[j];
        const float T = 0.5f*(mT[j] + mT[j+1]) - t;
        float dT = mT[j] - mT[j+1];
        dT *= dT;
        A += Z*dT;
        B += T*(T*dT);
        C +=   (T*dT);
        D += Z*(T*dT);
        E += dT;
    }
    b = (A*B + C*D) / (E*B + C*C);
    a = (E*b - A) / C;
    float f = b + PREDICTION_TIME*a;

    //LOGD("A=%f, B=%f, C=%f, D=%f, E=%f", A,B,C,D,E);
    //LOGD("%lld  %d  %f  %f", time, v, f, a);

    f = normalize(f);
    return f;
}

/*****************************************************************************/

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
        LOGD("%s, fd=%d", __PRETTY_FUNCTION__, sBmaFD);
        close(sBmaFD);
        sBmaFD = -1;
    }
}

static void enable_disable(int fd, uint32_t sensors, uint32_t mask)
{
    if (fd<0) return;
    short flags;
    mSensor = sensors;

    if (sensors & SENSOR_TYPE_ORIENTATION) {
        mask |= SENSOR_TYPE_ORIENTATION;
    }

  
    if (mask & SENSOR_TYPE_ORIENTATION) {
        flags = (sensors & SENSOR_TYPE_ORIENTATION) ? 1 : 0;
		
		if (flags) {
			if (ioctl(fd, ACCEL_IOC_ENABLE)) {
				LOGE("ACCEL_IOC_ENABLE error Orientation (%s)", strerror(errno));
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
        /*if (ioctl(fd, ECS_IOCTL_APP_SET_MFLAG, &flags) < 0) {
            LOGE("ECS_IOCTL_APP_SET_MFLAG error (%s)", strerror(errno));
        }*/
    }
    if (mask & SENSOR_TYPE_ACCELEROMETER) {
        flags = (sensors & SENSOR_TYPE_ACCELEROMETER) ? 1 : 0;

		if (flags) {
			if (ioctl(fd, ACCEL_IOC_ENABLE)) {
				LOGE("ACCEL_IOC_ENABLE error Accelerometer (%s)", strerror(errno));
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
        /*if (ioctl(fd, ECS_IOCTL_APP_SET_AFLAG, &flags) < 0) {
            LOGE("ECS_IOCTL_APP_SET_AFLAG error (%s)", strerror(errno));
        }*/
    }
}

static uint32_t read_sensors_state(int fd)
{
    if (fd<0) return 0;
    short flags;
    uint32_t sensors = 0;
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

uint32_t sensor_control_init()
{
    return SUPPORTED_SENSORS;
}

int sensor_control_open(struct sensors_control_device_t *dev)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	
	 LOGD("diyu type: sensor_control_open\n");
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
    
	LOGD("diyu type: sensor_control_activate\n");

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
            LOGD("sensors=%08x, real=%08x",
                sActiveSensors, read_sensors_state(fd));
             }
     	    return 0;
       } else {
 	    return -1;
       }
}

int sensor_control_delay(struct sensors_control_device_t *dev, int32_t ms)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	LOGD("diyu type: sensor_control_delay\n");
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
	LOGD("diyu type: sensor_control_wake\n");
	if (ctx) { 
		return -EWOULDBLOCK;
	} else {
		return -1;
	}
}

/*****************************************************************************/

#define MAX_NUM_SENSORS 2
static int sInputFD = -1;
/*static const int ID_O  = 1;
static const int ID_A  = 0;*/
static const int ID_O  = 0;
static const int ID_A  = 1;//original
static sensors_data_t sSensors[MAX_NUM_SENSORS];
static uint32_t sPendingSensors;

int sensor_data_open(struct sensors_data_device_t *dev, int fd)
{
   struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
   if (ctx) {
     int i;
    LMSInit();
    memset(&sSensors, 0, sizeof(sSensors));
		LOGD("diyu type: sensor_data_close\n");
    for (i=0 ; i<MAX_NUM_SENSORS ; i++) {
        // by default all sensors have high accuracy
        // (we do this because we don't get an update if the value doesn't
        // change).
        sSensors[i].vector.status = SENSOR_STATUS_ACCURACY_HIGH;
    }
    sPendingSensors = 0;
    sInputFD = dup(fd);
    LOGD("sensors_data_open: fd = %d", sInputFD);
        return 0;
    } else {
	return -1;
    }
}

int sensor_data_close(struct sensors_data_device_t *dev)
{
    struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	LOGD("diyu type: sensor_data_close\n");
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
		//LOGD("diyu type: pick_sensor\n");
        uint32_t i = 31 - __builtin_clz(mask);
        mask &= ~(1<<i);
        if (sPendingSensors & (1<<i)) {
            sPendingSensors &= ~(1<<i);
            *values = sSensors[i];
            values->sensor = (1<<i);
			/*
			LOGD("diyu %d [%f, %f, %f]", (1<<i),
                    values->vector.x,
                    values->vector.y,
                    values->vector.z);
			*/
            
            LOGD_IF(0, "%d [%f, %f, %f]", (1<<i),
                    values->vector.x,
                    values->vector.y,
                    values->vector.z);
            return (1<<i);
        }
    }
    LOGE("No sensor to return!!! sPendingSensors=%08x", sPendingSensors);
    // we may end-up in a busy loop, slow things down, just in case.
    usleep(100000);
    return -1;
}
float toDegreeX(int x) {
	float y = 180  - (float)(180 * x /15 );
//	float y = 180  - (float)(180 * x / 255);
	LOGD("diyu type: toDegreeX %f\n", y);
	return y;
}

#if 0
float toDegree(int x) {
	float y=0;
	#if 1
	float y = (float)(-1 * 90 * x / 15);
//	float y = (float)(-1 * 90 * x / 255);
    #else 
	if(x<0)
		y = multivalue_degree_xy_pair[64+x];
	else
		y = multivalue_degree_xy_pair[x];

		
	#endif 
	LOGD("diyu type: toDegree %f\n",y);
	return y;
}

float toDegreeZ(int x) {
	float y=0;
	#if 1
	float y = (float)(-1 * 90 * x / 15);
//	float y = (float)(-1 * 90 * x / 255);
    #else 
	if(x<0)
		 y = multivalue_degree_z_pair[64+x];
	else
		 y = multivalue_degree_z_pair[x];

	#endif 
	LOGD("diyu type: toDegree %f\n",y);
	return y;
}
#endif

		static int test_value = 90;

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
			//LOGD("diyu type: sPendingSensors\n");
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
	                /*LOGD("diyu type: %d code: %d value: %-5d time: %ds",
	                        event.type, event.code, event.value,
	                      (int)event.time.tv_sec);*/
	                switch (event.code) {

	                    case EVENT_TYPE_ACCEL_X:
	                        new_sensors |= SENSOR_TYPE_ACCELEROMETER;
							sSensors[ID_A].acceleration.x = (float)event.value * CONVERT_A_X;
							//LOGD("diyu type: CONVERT_A_X\n");
	                        break;
	                    case EVENT_TYPE_ACCEL_Y:
	                        new_sensors |= SENSOR_TYPE_ACCELEROMETER;
							sSensors[ID_A].acceleration.y = (float)event.value * CONVERT_A_Y;
							//LOGD("diyu type: CONVERT_A_Y\n");
	                        break;
	                    case EVENT_TYPE_ACCEL_Z:
	                        new_sensors |= SENSOR_TYPE_ACCELEROMETER;
							sSensors[ID_A].acceleration.z = (float)event.value * CONVERT_A_Z;
							//LOGD("diyu type: CONVERT_A_Z\n");
	                        break;

	                   case EVENT_TYPE_YAW:
					   	new_sensors |= SENSOR_TYPE_ORIENTATION ;
					   	 sSensors[ID_O].orientation.azimuth = event.value;
						 //LOGD("diyu type: EVENT_TYPE_YAW \n");
						 //sSensors[ID_O].orientation.azimuth = toDegreeZ(event.value);
						 #if 0
	                        new_sensors |= SENSOR_TYPE_ORIENTATION ;
							t = event.time.tv_sec*1000000LL +
	                                event.time.tv_usec*1000;
/*							t = event.time.tv_sec*1000000000LL +
	                                event.time.tv_usec*1000;*///original
	                        sSensors[ID_O].orientation.azimuth = 
	                            (mSensor & SENSOR_TYPE_ORIENTATION) ?
	                                    LMSFilter(t, toDegree(event.value)) : event.value;
						#endif
/*	                        sSensors[ID_O].orientation.azimuth = 
	                            (mSensor & SENSOR_TYPE_ORIENTATION) ?
	                                    LMSFilter(t, event.value) : event.value;							*/
							//LOGD("diyu type: EVENT_TYPE_YAW %f \n", sSensors[ID_O].orientation.azimuth );
	                        
/*							sSensors[ID_O].orientation.azimuth = 
	                            (sensors_of_interest & SENSOR_TYPE_ORIENTATION) ?
	                                    LMSFilter(t, event.value) : event.value;*/
	                        break;
	                    case EVENT_TYPE_PITCH:
	                        new_sensors |= SENSOR_TYPE_ORIENTATION ;
	                        //sSensors[ID_O].orientation.pitch = toDegree(event.value);
							sSensors[ID_O].orientation.pitch = event.value;
							//LOGD("diyu type: EVENT_TYPE_PITCH %f \n",	sSensors[ID_O].orientation.pitch );

	                        break;
	                    case EVENT_TYPE_ROLL:
	                        new_sensors |= SENSOR_TYPE_ORIENTATION ;
	                        //sSensors[ID_O].orientation.roll = toDegree(event.value);
							sSensors[ID_O].orientation.roll = event.value;
							//LOGD("diyu type: EVENT_TYPE_ROLL %f\n", sSensors[ID_O].orientation.roll );
	                        break;

	                    case EVENT_TYPE_STEP_COUNT:
	                        // step count (only reported in MODE_FFD)
	                        // we do nothing with it for now.
	                        break;
	                    case EVENT_TYPE_ACCEL_STATUS:
	                        // accuracy of the calibration (never returned!)
	                        //LOGD("G-Sensor status %d", event.value);
	                        break;
	                    case EVENT_TYPE_ORIENT_STATUS:
	                        // accuracy of the calibration
	                        v = (uint32_t)(event.value & SENSOR_STATE_MASK);
	                        LOGD_IF(sSensors[ID_O].orientation.status != (uint8_t)v,
	                                "M-Sensor status %d", v);
	                        sSensors[ID_O].orientation.status = (uint8_t)v;
							//LOGD("diyu type: EVENT_TYPE_ORIENT_STATUS\n");
	                        break;
	                }
	            } else if (event.type == EV_SYN) {
					//LOGD("diyu type: event.type == EV_SYN\n");
	                if (new_sensors) {
	                    sPendingSensors = new_sensors;
	                    int64_t t = event.time.tv_sec*1000000000LL +
	                            event.time.tv_usec*1000;
						//LOGD("diyu type:  if (new_sensors) {\n");
	                    while (new_sensors) {
	                        uint32_t i = 31 - __builtin_clz(new_sensors);
	                        new_sensors &= ~(1<<i);
	                        sSensors[i].time = t;
	                    }
	                    return pick_sensor(values);
	                }
	            }
	        }
	    }

	}
}

int sensors_get_sensors(struct sensors_module_t* module, struct sensor_t const** sensors) {
	*sensors = &LGE_SENSOR;
//    return SUPPORTED_SENSORS;
	LOGD("diyu type: sensors_get_sensors\n");

    return 1;
}

static int close_sensors(struct hw_device_t *dev) 
{
	struct sensors_context_t* ctx = (struct sensors_context_t*)dev;
	LOGD("diyu type: close_sensors\n");
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
	LOGD("diyu type: open_sensors\n");

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
