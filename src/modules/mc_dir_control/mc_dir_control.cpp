/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file mc_dir_control.cpp
 * Direct motor control mode.
 *
 * @author JunEn Low <jelow@stanford.edu>
 */

#include "mc_dir_control.hpp"

using namespace matrix;

MulticopterDirectControl::MulticopterDirectControl() :
        ModuleParams(nullptr),
        WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
        _actuators_0_pub(ORB_ID(actuator_controls_0)),
        _loop_perf(perf_alloc(PC_ELAPSED, "Multicopter Direct Control"))
{
}

MulticopterDirectControl::~MulticopterDirectControl()
{
    perf_free(_loop_perf);
}

bool
MulticopterDirectControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterDirectControl::publish_actuator_controls()
{
	// zero actuators if not armed
	if (_vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		for (uint8_t i = 0 ; i < 4 ; i++) {
			_actuators.control[i] = 0.00f;
		}

	} else {
		_actuators.control[0] = _rc_channel.channels[0];
		_actuators.control[1] = 0.00f;
		_actuators.control[2] = 0.00f;
		_actuators.control[3] = 0.00f;
	}

	// note: _actuators.timestamp_sample is set in AirshipAttitudeControl::Run()
	_actuators.timestamp = hrt_absolute_time();

	_actuators_0_pub.publish(_actuators);
}

void MulticopterDirectControl::Run()
{
    if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_attitude_s v_att;

	if (_att_sub.update(&v_att)) {
		publish_actuator_controls();

		/* check for updates to rc_channel topic */
		if (_rc_channels_sub.updated()) {
			_rc_channels_sub.update(&_rc_channel);
		}
		/* check for updates in vehicle status topic */
		if (_vehicle_status_sub.updated()) {
			_vehicle_status_sub.update(&_vehicle_status);
		}
	}

}

int MulticopterDirectControl::task_spawn(int argc, char *argv[])
{
	MulticopterDirectControl *instance = new MulticopterDirectControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int MulticopterDirectControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterDirectControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements direct motor control via a wrench command (Fz,Mx,My,Mz)

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_dir_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_dir_control_main(int argc, char *argv[])
{
	return MulticopterDirectControl::main(argc, argv);
}
