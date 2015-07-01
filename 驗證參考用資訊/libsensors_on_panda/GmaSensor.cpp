/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "Sensors"
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "GmaSensor.h"
#define ACC_INPUT_NAME  "gma302"//"accelerometer"
#define GMA_UNIT_CONVERSION(value)		((value) *(GRAVITY_EARTH / (1024.0f)))//(1004544.0f)))  //(1024)*981 becomes 1g = 1004544
#define ACC_DEV_PATH_NAME    "/dev/gma303"

int write_sys_attribute(const char *path, const char *value, int bytes)
{
    int ifd, amt;

	ifd = open(path, O_WRONLY);
    if (ifd < 0) {
        ALOGE("Write_attr failed to open %s (%s)",path, strerror(errno));
        return -1;
	}

    amt = write(ifd, value, bytes);
	amt = ((amt == -1) ? -errno : 0);
	ALOGE_IF(amt < 0, "Write_int failed to write %s (%s)",
		path, strerror(errno));
    close(ifd);
	return amt;
}
/*****************************************************************************/
GmaSensor::GmaSensor()
    : SensorBase(ACC_DEV_PATH_NAME, ACC_INPUT_NAME),
      mEnabled(1),
      mDelay(10000),
      //mLayout(1),
      mInputReader(32),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_A;
    mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
    
    if (data_fd >= 0) {
        strcpy(input_sysfs_path, "/sys/class/input/");
        strcat(input_sysfs_path, input_name);
        strcat(input_sysfs_path, "/device/");
        input_sysfs_path_len = strlen(input_sysfs_path);
		ALOGD("GmaSensor: sysfs_path=%s", input_sysfs_path);
    } else {
		input_sysfs_path[0] = '\0';
		input_sysfs_path_len = 0;
	}
	
    open_device();
}

GmaSensor::~GmaSensor() {
    if (mEnabled) {
        setEnable(0, 0);
    }

    close_device();
}

int GmaSensor::setInitialState() {
    struct input_absinfo absinfo;

	if (mEnabled) {
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_X), &absinfo)) {
			mPendingEvent.acceleration.x = GMA_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Y), &absinfo)) {
			mPendingEvent.acceleration.y = GMA_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Z), &absinfo)) {
			mPendingEvent.acceleration.z = GMA_UNIT_CONVERSION(absinfo.value);
		}
	}
	return 0;
}

bool GmaSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int GmaSensor::setEnable(int32_t handle, int enabled) {
    int err = 0;
	char buffer[2]={0,0};
	// handle check 
	if (handle != ID_A) {
		ALOGE("GmaSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}
	buffer[0]= enabled ? '1':'0';
	//ALOGD("GmaSensor: enabled = %s", buffer);
	strcpy(&input_sysfs_path[input_sysfs_path_len], "enable");
	err = write_sys_attribute(input_sysfs_path, buffer, 1);
	if (err != 0) 
		return err;
	
    return err;
}

int GmaSensor::setDelay(int32_t handle, int64_t delay_ns)
{
	int err = 0;
	int32_t us; 
	char buffer[16];
	int bytes;
	/* handle check */
	if (handle != ID_A) {
		ALOGE("GmaSensor: Invalid handle (%d)", handle);
		return -EINVAL;
	}
	if (mDelay != delay_ns) {
		us = (int32_t)(delay_ns / 1000000);

    	strcpy(&input_sysfs_path[input_sysfs_path_len], "delay");
   		bytes = sprintf(buffer, "%d", (int)us);
		err = write_sys_attribute(input_sysfs_path, buffer, bytes);
		if (err == 0) {
			mDelay = delay_ns;
			ALOGD("GmaSensor: Control set delay %f ms requetsed, ",
				delay_ns/1000000.0f);
		}
	}
	return err;
}

int64_t GmaSensor::getDelay(int32_t handle)
{
    return (handle == ID_A) ? mDelay : 0;
}

int GmaSensor::getEnable(int32_t handle)
{
    return (handle == ID_A) ? mEnabled : 0;
}

int GmaSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			float value = event->value;
			if (event->code == EVENT_TYPE_ACCEL_X) {
				mPendingEvent.acceleration.x = GMA_UNIT_CONVERSION(value);
			} else if (event->code == EVENT_TYPE_ACCEL_Y) {
				mPendingEvent.acceleration.y = GMA_UNIT_CONVERSION(value);
			} else if (event->code == EVENT_TYPE_ACCEL_Z) {
				mPendingEvent.acceleration.z = GMA_UNIT_CONVERSION(value);
			}
		} else if (type == EV_SYN) {
			mPendingEvent.timestamp = timevalToNano(event->time);
			if (mEnabled) {
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
		} else {
			ALOGE("GmaSensor: unknown event (type=%d, code=%d)",
					type, event->code);
		}
        mInputReader.next();
    }

    return numEventReceived;
}
