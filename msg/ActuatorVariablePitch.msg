# Variable pitch control message
uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

uint8 ACTUATOR_FUNCTION_VPP1 = 151

uint8 NUM_CONTROLS = 12
float32[12] control # range: [0, 1], where 1 means maximum positive,
		    # -1 maximum negative
                    # and NaN maps to disarmed
