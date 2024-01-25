/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <uORB/topics/actuator_variable_pitch.h>


/**
 * @brief Functions: VariablePitch (Used for actuating a variable pitch propeller)
 *
 */
class FunctionVariablePitch : public FunctionProviderBase
{
public:

	FunctionVariablePitch(const Context &context): _topic(&context.work_item, ORB_ID(actuator_variable_pitch))
	{
		for (int i = 0; i < actuator_variable_pitch_s::NUM_CONTROLS; ++i) {
			_data.control[i] = NAN;
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionVariablePitch(context); }

	void update() override { _topic.update(&_data); }

	bool allowPrearmControl() const override { return true; }

	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Variable_Pitch_Mechanism1]; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

private:
	uORB::SubscriptionCallbackWorkItem _topic;
	actuator_variable_pitch_s _data{};
};
