/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef MODULE_NAME
#define MODULE_NAME "rock5b_sysfs_pwm_out"
#endif

#include "board_pwm_out.h"

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <px4_platform_common/log.h>

#ifdef __PX4_EVL4
#include <evl/syscall.h>
#include <linux/pwm.h>
#include <string.h>
#include <px4_platform_common/evl_helper.h>
#endif

using namespace pwm_out;

const char RockSysfsPWMOut::_device[] = "/sys/class/pwm/pwmchip";

#if defined(__PX4_EVL4)
const char RockSysfsPWMOut::_oob_device[] = "/dev/pwm";
#endif

RockSysfsPWMOut::RockSysfsPWMOut(int max_num_outputs)
{
	if (max_num_outputs > MAX_NUM_PWM) {
		PX4_WARN("number of outputs too large. Setting to %i", MAX_NUM_PWM);
		max_num_outputs = MAX_NUM_PWM;
	}

	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		_pwm_fd[i] = -1;
	}

	_pwm_num = max_num_outputs;
}

RockSysfsPWMOut::~RockSysfsPWMOut()
{
	for (int i = 0; i < MAX_NUM_PWM; ++i) {
		if (_pwm_fd[i] != -1) {
			::close(_pwm_fd[i]);
		}
	}
}

int RockSysfsPWMOut::init()
{
	int i;
	char path[128];

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s%d/export", _device, i>0?i+1:0); // 1 is pwm fan
		if (pwm_write_sysfs(path, 0) < 0) {
			PX4_ERR("PWM export failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s%d/pwm0/period", _device, i>0?i+1:0);
		if (pwm_write_sysfs(path, (int)1e9 / FREQUENCY_PWM)) {
			PX4_ERR("PWM period failed");
		}
	}


	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s%d/pwm0/polarity", _device, i>0?i+1:0);
		if (pwm_write_sysfs_str(path, "normal", sizeof("normal")) < 0) {
			PX4_ERR("PWM polarity set failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s%d/pwm0/enable", _device, i>0?i+1:0);
		if (pwm_write_sysfs(path, 1) < 0) {
			PX4_ERR("PWM enable failed");
		}
	}

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s%d/pwm0/duty_cycle", _device, i>0?i+1:0);
		_pwm_fd[i] = ::open(path, O_WRONLY | O_CLOEXEC);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open duty_cycle.");
			return -errno;
		}
	}

	return 0;
}

#ifdef __PX4_EVL4
int RockSysfsPWMOut::oob_init()
{
	int i;
	char path[128];

	for (i = 0; i < _pwm_num; ++i) {
		::snprintf(path, sizeof(path), "%s%d", _oob_device, i>0?i+1:0); // 1 is pwm fan
		_pwm_fd[i] = ::open(path, O_WRONLY | O_OOB);

		if (_pwm_fd[i] == -1) {
			PX4_ERR("PWM: Failed to open %s. error: %s", path, strerror(errno));
			return -errno;
		}
	}

	return 0;
}
#endif

int RockSysfsPWMOut::send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	char data[16];

	if (num_outputs > _pwm_num) {
		num_outputs = _pwm_num;
	}

	int ret = 0;

	//convert this to duty_cycle in ns
	for (int i = 0; i < num_outputs; ++i) {
		int n = ::snprintf(data, sizeof(data), "%u", pwm[i] * 1000);
		int write_ret = ::write(_pwm_fd[i], data, n);

		if (n != write_ret) {
			ret = -1;
		}
	}

	return ret;
}

#ifdef __PX4_EVL4
int RockSysfsPWMOut::oob_send_output_pwm(const uint16_t *pwm, int num_outputs)
{
	if (num_outputs > _pwm_num) {
		num_outputs = _pwm_num;
	}

	int ret = 0;
	struct pwm_state_request req;

	req.enabled = true;
        req.polarity = PWM_UAPI_POLARITY_NORMAL;
        req.period = (int)1e9 / FREQUENCY_PWM;

	//convert this to duty_cycle in ns
	for (int i = 0; i < num_outputs; ++i) {
		req.duty_cycle = pwm[i] * 1000;
		ret = ioctl(_pwm_fd[i], PWM_SET_STATE_IOCTL, &req);

		if (ret < 0) {
			PX4_ERR("PWM: Failed to oob_set duty cycle. error: %s", strerror(errno));
			return -errno;
		}
	}

	return ret;
}
#endif

int RockSysfsPWMOut::pwm_write_sysfs(char *path, int value)
{
	int fd = ::open(path, O_WRONLY | O_CLOEXEC);
	int n;
	char data[16];

	if (fd == -1) {
		return -errno;
	}

	n = ::snprintf(data, sizeof(data), "%u", value);

	if (n > 0) {
		n = ::write(fd, data, n);	// This n is not used, but to avoid a compiler error.
	}

	::close(fd);

	return 0;
}

int RockSysfsPWMOut::pwm_write_sysfs_str(char *path,const char* s, int n)
{
	int fd = ::open(path, O_WRONLY | O_CLOEXEC);

	if (fd == -1) {
		return -errno;
	}

	n = ::write(fd, s, n);	// This n is not used, but to avoid a compiler error.


	::close(fd);

	return 0;
}
