/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "linux_pwm_out.hpp"

#include <board_pwm_out.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/sem.hpp>

#ifdef __PX4_EVL4
#include <px4_platform_common/evl_helper.h>
#include <evl/clock.h>

#define OOB_RUN 0x1
#define OOB_STOP 0x2

static void *oob_send_trampoline(void *arg);
#endif

using namespace pwm_out;

LinuxPWMOut::LinuxPWMOut() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
}

LinuxPWMOut::~LinuxPWMOut()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
#ifdef __PX4_EVL4
	if (oob_thread_running) {
		int eret;
		__Tcall_assert(eret, evl_post_flags(&flags, OOB_STOP));
	}
#endif
	delete _pwm_out;
}

int LinuxPWMOut::init()
{
	_pwm_out = new BOARD_PWM_OUT_IMPL(MAX_ACTUATORS);

#ifdef __PX4_EVL4
	int ret = _pwm_out->oob_init();
#else
	int ret = _pwm_out->init();
#endif

	if (ret != 0) {
		PX4_ERR("PWM output init failed");
		delete _pwm_out;
		_pwm_out = nullptr;
		return ret;
	}
#ifdef __PX4_EVL4
	int eret;
	pthread_t oob_send;

	__Tcall_assert(eret, evl_create_flags(&flags, EVL_CLOCK_MONOTONIC, 0, EVL_CLONE_PUBLIC, "LinuxPWMOut_flags"));
	pthread_create(&oob_send, nullptr, oob_send_trampoline, this);
	oob_thread_running = true;
#endif

	ScheduleNow();

	return ret;
}

int LinuxPWMOut::task_spawn(int argc, char *argv[])
{
	LinuxPWMOut *instance = new LinuxPWMOut();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool LinuxPWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				unsigned num_outputs, unsigned num_control_groups_updated)
{
#ifdef __PX4_EVL4
	int eret;

	inner_arg.outputs = outputs;
	inner_arg.num_outputs = num_outputs;
	__Tcall_assert(eret, evl_post_flags(&flags, OOB_RUN));
#else
	_pwm_out->send_output_pwm(outputs, num_outputs);
#endif
	return true;
}

void LinuxPWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	_mixing_output.updateSubscriptions(false);

	perf_end(_cycle_perf);
}

int LinuxPWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LinuxPWMOut::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();
	return 0;
}

int LinuxPWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Linux PWM output driver with board-specific backend implementation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("linux_pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

#ifdef __PX4_EVL4
static void *oob_send_trampoline(void *arg)
{
	LinuxPWMOut *pwm_out = (LinuxPWMOut *)arg;
	int rbits = 0;
	int eret;

	__attach_and_setsched(SCHED_FIFO, SCHED_PRIORITY_FAST_DRIVER, "oob_send_trampoline", nullptr);

	while (1) {
		__Tcall_assert(eret, evl_wait_some_flags(&pwm_out->flags, OOB_RUN | OOB_STOP, &rbits));

		if (rbits & OOB_STOP) {
			__Tcall_assert(eret, evl_close_flags(&pwm_out->flags));
			evl_detach_self();
			break;
		}

		PWMOutBase* pwm_out_base = pwm_out->get_pwm_out();
		pwm_out_base->oob_send_output_pwm(pwm_out->inner_arg.outputs, pwm_out->inner_arg.num_outputs);
	}

	return nullptr;
}
#endif

extern "C" __EXPORT int linux_pwm_out_main(int argc, char *argv[])
{
	return LinuxPWMOut::main(argc, argv);
}
