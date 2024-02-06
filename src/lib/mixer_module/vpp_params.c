/**
 * @file vpp_params.c
 *
 * Parameters for variable pitch propellers.
 *
 */


/**
 * Coefficient of VPP control equation.
 *
 * @decimal 7
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_P00, 0.8428f);


/**
 * Coefficient of VPP control equation.
 * @decimal 7
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_P10, -2.515f);


/**
 * Coefficient of VPP control equation.
 * @decimal 7
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_P01, 0.03881f);


/**
 * Coefficient of VPP control equation.
 * @decimal 7
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_P20, 1.396f);


/**
 * Coefficient of VPP control equation.
 * @decimal 7
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_P11, -0.0168f);


/**
 * Coefficient of VPP control equation.
 * @decimal 7
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_P02, -0.0001029f);


/**
 * Enable VPP control.
 *
 * @boolean
 * @group Variable Pitch
 */
PARAM_DEFINE_INT32(VPP_EN, 0);


/**
 * VPP control value when VPP_EN is 0.
 *
 * Control signal when VPP is not computed using algorithm.
 * @min 0
 * @max 1
 * @decimal 2
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_OFF_VAL, 0.5f);

/**
 * Airspeed threshold for VPP algorithm.
 *
 * Applies only when VPP_EN is 1. Airspeed value below which control is constant value equal to VPP_LOW_AS_VAL.
 * Above this value control is calculated using VPP control equation.
 *
 * @min 0
 * @decimal 2
 * @unit m/s
 * @group Variable Pitch
 */
PARAM_DEFINE_FLOAT(VPP_ASPD_THRSHLD, 9.0f);
