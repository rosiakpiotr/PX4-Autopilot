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

#pragma once

#include "FunctionProviderBase.hpp"

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/actuator_variable_pitch.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/Publication.hpp>


/**
 * Functions: Motor1 ... MotorMax
 */
class FunctionMotors : public FunctionProviderBase
{
public:
	static_assert(actuator_motors_s::NUM_CONTROLS == (int)OutputFunction::MotorMax - (int)OutputFunction::Motor1 + 1,
		      "Unexpected num motors");

	static_assert(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 == (int)OutputFunction::Motor1, "Unexpected motor idx");

	FunctionMotors(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_motors)),
		_thrust_factor(context.thrust_factor),
		p00(context.vpp_p00),
		p10(context.vpp_p10),
		p01(context.vpp_p01),
		p20(context.vpp_p20),
		p11(context.vpp_p11),
		p02(context.vpp_p02),
		vpp_enabled(context.vpp_enabled),
		vpp_off_val(context.vpp_off_val)
	{
		_actuator_variable_pitch_pub.advertise();

		for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
		}
	}

	virtual ~FunctionMotors()
	{
		_actuator_variable_pitch_pub.unadvertise();
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionMotors(context); }

	void update() override
	{
		airspeed_validated_s airspeed;
		bool airspeed_updated = _airspeed_validated_sub.update(&airspeed);

		vehicle_status_s vehicle_status;
		bool vehicle_status_updated = _vehicle_status_sub.update(&vehicle_status);

		if (vehicle_status_updated) {
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_has_zero_airspeed = true;

			} else {
				_has_zero_airspeed = false;
			}
		}

		if (_has_zero_airspeed) {
			_airspeed = 0.f;

		} else if (airspeed_updated && airspeed.airspeed_sensor_measurement_valid) {
			_airspeed = airspeed.true_airspeed_m_s;
		}

		bool data_updated = _topic.update(&_data);

		if (data_updated) {
			updateValues(_data.reversible_flags, _thrust_factor, _data.control, actuator_motors_s::NUM_CONTROLS);
		}

		if (airspeed_updated || data_updated || vehicle_status_updated) {
			updateVpp();
		}
	}

	void updateVpp()
	{
		actuator_variable_pitch_s actuator_vpp;
		actuator_vpp.timestamp = hrt_absolute_time();
		actuator_vpp.timestamp_sample = _data.timestamp_sample;

		for (int i = 0; i < actuator_variable_pitch_s::NUM_CONTROLS; ++i) {
			if (!vpp_enabled) {
				actuator_vpp.control[i] = vpp_off_val;
				continue;
			}

			if (isnan(_data.control[i])) {
				actuator_vpp.control[i] = NAN;
				continue;
			}

			float motor_control = _data.control[i];
			float pitch_control = p00 + p10 * motor_control + p01 * _airspeed +
					      p20 * motor_control * motor_control + p11 * motor_control * _airspeed +
					      p02 * _airspeed * _airspeed;
			actuator_vpp.control[i] = math::constrain(pitch_control, 0.f, 1.f);
		}

		_actuator_variable_pitch_pub.publish(actuator_vpp);
	}

	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Motor1]; }

	bool allowPrearmControl() const override { return false; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	static inline void updateValues(uint32_t reversible, float thrust_factor, float *values, int num_values)
	{
		if (thrust_factor > 0.f && thrust_factor <= 1.f) {
			// thrust factor
			//  rel_thrust = factor * x^2 + (1-factor) * x,
			const float a = thrust_factor;
			const float b = (1.f - thrust_factor);

			// don't recompute for all values (ax^2+bx+c=0)
			const float tmp1 = b / (2.f * a);
			const float tmp2 = b * b / (4.f * a * a);

			for (int i = 0; i < num_values; ++i) {
				float control = values[i];

				if (control > 0.f) {
					values[i] = -tmp1 + sqrtf(tmp2 + (control / a));

				} else if (control < -0.f) {
					values[i] =  tmp1 - sqrtf(tmp2 - (control / a));

				} else {
					values[i] = 0.f;
				}
			}
		}

		for (int i = 0; i < num_values; ++i) {
			if ((reversible & (1u << i)) == 0) {
				if (values[i] < -FLT_EPSILON) {
					values[i] = NAN;

				} else {
					// remap from [0, 1] to [-1, 1]
					values[i] = values[i] * 2.f - 1.f;
				}
			}
		}
	}

	bool reversible(OutputFunction func) const override { return _data.reversible_flags & (1u << ((int)func - (int)OutputFunction::Motor1)); }

private:
	uORB::SubscriptionCallbackWorkItem _topic;
	actuator_motors_s _data{};

	uORB::Publication<actuator_variable_pitch_s> _actuator_variable_pitch_pub{ORB_ID(actuator_variable_pitch)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	float _airspeed;
	bool _has_zero_airspeed;

	const float &_thrust_factor;

	// VPP control equation coefficients
	const float &p00;
	const float &p10;
	const float &p01;
	const float &p20;
	const float &p11;
	const float &p02;
	const int &vpp_enabled;
	const float &vpp_off_val;
};
