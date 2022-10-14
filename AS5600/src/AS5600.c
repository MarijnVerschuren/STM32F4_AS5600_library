/// includes
#include "as5600.h"

/// AS5600 functions
/* initialization */
AS5600_TypeDef* AS5600_new(void) { return (AS5600_TypeDef*)calloc(1, sizeof(AS5600_TypeDef)); }
HAL_StatusTypeDef AS5600_init(AS5600_TypeDef* handle) {
	// set all unspecified fields to their default
	if (!(handle->i2c_timeout))					{ handle->i2c_timeout = AS5600_I2C_TIMEOUT_DEFAULT; }
	if (!(handle->positive_rotation_direction))	{ handle->positive_rotation_direction = AS5600_DIR_CW; }
	if (!(handle->low_power_mode))				{ handle->low_power_mode = AS5600_POWER_MODE_DEFAULT; }
	if (!(handle->hysteresis))					{ handle->hysteresis = AS5600_HYSTERESIS_DEFAULT; }
	if (!(handle->output_mode))					{ handle->output_mode = AS5600_OUTPUT_STAGE_DEFAULT; }
	if (!(handle->PWM_frequency))				{ handle->PWM_frequency = AS5600_PWM_FREQUENCY_DEFAULT; }
	if (!(handle->slow_filter))					{ handle->slow_filter = AS5600_SLOW_FILTER_DEFAULT; }
	if (!(handle->fast_filter_threshold))		{ handle->fast_filter_threshold = AS5600_FAST_FILTER_DEFAULT; }
	if (!(handle->watchdog_timer))				{ handle->watchdog_timer = AS5600_WATCHDOG_DEFAULT; }

	if (AS5600_set_low_power_mode(handle, handle->low_power_mode) != HAL_OK)					{ return HAL_ERROR; }
	if (AS5600_set_hysteresis(handle, handle->hysteresis) != HAL_OK)							{ return HAL_ERROR; }
	if (AS5600_set_output_mode(handle, handle->output_mode, handle->PWM_frequency) != HAL_OK)	{ return HAL_ERROR; }
	if (AS5600_set_slow_filter(handle, handle->slow_filter) != HAL_OK)							{ return HAL_ERROR; }
	if (AS5600_set_fast_filter_threshold(handle, handle->fast_filter_threshold) != HAL_OK)		{ return HAL_ERROR; }
	if (AS5600_set_watchdog_timer(handle, handle->watchdog_timer) != HAL_OK)					{ return HAL_ERROR; }
	
	if (AS5600_write_config_register(handle) != HAL_OK)											{ return HAL_ERROR; }
    uint8_t magnet_status = 0;
	if (AS5600_get_magnet_status(handle, &magnet_status) != HAL_OK)								{ return HAL_ERROR; }
	if (!(magnet_status & AS5600_MAGNET_DETECTED) \
		|| (magnet_status & AS5600_AGC_MIN_GAIN_OVERFLOW) \
		|| (magnet_status & AS5600_AGC_MAX_GAIN_OVERFLOW))										{ return HAL_ERROR; }
	return HAL_OK;
}

/* setters */
HAL_StatusTypeDef AS5600_set_start_position(AS5600_TypeDef* const handle, const uint16_t position) {
	uint8_t data[2];
	data[0] = (uint8_t)((position & AS5600_12_BIT_MASK) >> 8);	// zero out upper four bits of argument and shift out lower four bits
	data[1] = (uint8_t)position;
	return HAL_I2C_Mem_Write(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_ZPOS_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, handle->i2c_timeout);
}
HAL_StatusTypeDef AS5600_set_stop_position(AS5600_TypeDef* const handle, const uint16_t position) {
	uint8_t data[2];
	data[0] = (uint8_t)((position & AS5600_12_BIT_MASK) >> 8);	// zero out upper four bits of argument and shift out lower four bits
	data[1] = (uint8_t)position;
	return HAL_I2C_Mem_Write(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_MPOS_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, handle->i2c_timeout);
}
HAL_StatusTypeDef AS5600_set_max_angle(AS5600_TypeDef* const handle, const uint16_t angle) {
	uint8_t data[2];
	data[0] = (uint8_t)((angle & AS5600_12_BIT_MASK) >> 8);	// zero out upper four bits of argument and shift out lower four bits
	data[1] = (uint8_t)angle;
	return HAL_I2C_Mem_Write(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_MANG_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, handle->i2c_timeout);
}
HAL_StatusTypeDef AS5600_set_positive_rotation_direction(AS5600_TypeDef* const handle, const uint8_t direction) {
	switch (direction) {
	case AS5600_DIR_CW:		HAL_GPIO_WritePin(handle->dir_port, handle->dir_pin, GPIO_PIN_RESET);
	case AS5600_DIR_CCW:	HAL_GPIO_WritePin(handle->dir_port, handle->dir_pin, GPIO_PIN_SET);
	default:				return HAL_ERROR;	// invalid direction
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_set_low_power_mode(AS5600_TypeDef* const handle, const uint8_t mode) {
	switch (mode) {
	case AS5600_POWER_MODE_NOM:
		handle->config_register[1] &= ~((1UL << 1) | (1UL << 0));
		break;
	case AS5600_POWER_MODE_LPM1:
		handle->config_register[1] |= (1UL << 0);
		handle->config_register[1] &= ~(1UL << 1);
	break;
	case AS5600_POWER_MODE_LPM2:
		handle->config_register[1] |= (1UL << 1);
		handle->config_register[1] &= (1UL << 0);
		break;
	case AS5600_POWER_MODE_LPM3:
		handle->config_register[1] |= ((1UL << 1) | (1UL << 0));
		break;
	default: return HAL_ERROR;  // invalid mode
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_set_hysteresis(AS5600_TypeDef* const handle, const uint8_t hysteresis) {
	switch (hysteresis) {
	case AS5600_HYSTERESIS_OFF:
		handle->config_register[1] &= ~((1UL << 3) | (1UL << 2));
		break;
	case AS5600_HYSTERESIS_1LSB:
		handle->config_register[1] |= (1UL << 2);
		handle->config_register[1] &= ~(1UL << 3);
		break;
	case AS5600_HYSTERESIS_2LSB:
		handle->config_register[1] &= ~(1UL << 2);
		handle->config_register[1] |= (1UL << 3);
		break;
	case AS5600_HYSTERESIS_3LSB:
		handle->config_register[1] |= ((1UL << 3) | (1UL << 2));
		break;
	default: return HAL_ERROR;	// invalid hysteresis mode
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_set_output_mode(AS5600_TypeDef* const handle, const uint8_t mode, uint8_t frequency) {
	uint8_t pwm = 0;
	switch (mode) {
	case AS5600_OUTPUT_STAGE_FULL:
		handle->config_register[1] &= ~((1UL << 5) | (1UL << 4));
		break;
	case AS5600_OUTPUT_STAGE_REDUCED:
		handle->config_register[1] |= (1UL << 4);
		handle->config_register[1] &= ~(1UL << 5);
		break;
	case AS5600_OUTPUT_STAGE_PWM:
		handle->config_register[1] &= (1UL << 4);
		handle->config_register[1] |= (1UL << 5);
		pwm = 1; break;
	default: return HAL_ERROR;	// invalid output mode
	}
	if (pwm) {
		switch (frequency) {
		case AS5600_PWM_FREQUENCY_115HZ:
			handle->config_register[1] &= ~((1UL << 7) | (1UL << 6));
			break;
		case AS5600_PWM_FREQUENCY_230HZ:
			handle->config_register[1] |= (1UL << 6);
			handle->config_register[1] &= ~(1UL << 7);
			break;
		case AS5600_PWM_FREQUENCY_460HZ:
			handle->config_register[1] &= ~(1UL << 6);
			handle->config_register[1] |= (1UL << 7);
			break;
		case AS5600_PWM_FREQUENCY_920HZ:
			handle->config_register[1] |= ((1UL << 7) | (1UL << 6));
			break;
		default: return HAL_ERROR;	// invalid PWM frequency
		}
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_set_slow_filter(AS5600_TypeDef* const handle, const uint8_t mode) {
	switch (mode) {
	case AS5600_SLOW_FILTER_16X:
		handle->config_register[0] &= ~((1UL << 1) | (1UL << 0));
		break;
	case AS5600_SLOW_FILTER_8X:
		handle->config_register[0] |= (1UL << 0);
		handle->config_register[0] &= ~(1UL << 1);
		break;
	case AS5600_SLOW_FILTER_4X:
		handle->config_register[0] &= ~(1UL << 0);
		handle->config_register[0] |= (1UL << 1);
		break;
	case AS5600_SLOW_FILTER_2X:
		handle->config_register[0] |= ((1UL << 1) | (1UL << 0));
		break;
	default: return HAL_ERROR;	// invalid slow filter mode
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_set_fast_filter_threshold(AS5600_TypeDef* const handle, const uint8_t threshold) {
	switch (threshold) {
	case AS5600_FAST_FILTER_SLOW_ONLY:
		handle->config_register[0] &= ~((1UL << 4) | (1UL << 3) | (1UL << 2));
		break;
	case AS5600_FAST_FILTER_6LSB:
		handle->config_register[0] &= ~((1UL << 4) | (1UL << 3));
		handle->config_register[0] |= (1UL << 2);
		break;
	case AS5600_FAST_FILTER_7LSB:
		handle->config_register[0] &= ~((1UL << 4) | (1UL << 2));
		handle->config_register[0] |= (1UL << 3);
		break;
	case AS5600_FAST_FILTER_9LSB:
		handle->config_register[0] &= ~(1UL << 4);
		handle->config_register[0] |= ((1UL << 3) | (1UL << 2));
		break;
	case AS5600_FAST_FILTER_18LSB:
		handle->config_register[0] &= ~((1UL << 3) | (1UL << 2));
		handle->config_register[0] |= (1UL << 4);
		break;
	case AS5600_FAST_FILTER_21LSB:
		handle->config_register[0] &= ~(1UL << 3);
		handle->config_register[0] |= ((1UL << 4) | (1UL << 2));
		break;
	case AS5600_FAST_FILTER_24LSB:
		handle->config_register[0] &= ~(1UL << 2);
		handle->config_register[0] |= ((1UL << 4) | (1UL << 3));
		break;
	case AS5600_FAST_FILTER_10LSB:
		handle->config_register[0] |= ((1UL << 4) | (1UL << 3) | (1UL << 2));
		break;
	default: return HAL_ERROR;	// invalid fast filter mode
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_set_watchdog_timer(AS5600_TypeDef* const handle, const uint8_t mode) {
	switch (mode) {
	case AS5600_WATCHDOG_OFF:
		handle->config_register[0] &= ~(1UL << 6);
		break;
	case AS5600_WATCHDOG_ON:
		handle->config_register[0] |= (1UL << 6);
		break;
	default: return HAL_ERROR;	// invalid watchdog state
	}
	return HAL_OK;
}
HAL_StatusTypeDef AS5600_write_config_register(AS5600_TypeDef* const handle) { return HAL_I2C_Mem_Write(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_CONF_HIGH, I2C_MEMADD_SIZE_8BIT, handle->config_register, 2, handle->i2c_timeout); }
/* getters */
HAL_StatusTypeDef AS5600_get_rawAngle(AS5600_TypeDef* const handle, uint16_t* const angle) {
	uint8_t data[2] = {0};
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_RAW_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, handle->i2c_timeout);
	*angle = ((data[0] << 8) | data[1]);
	return status;
}
HAL_StatusTypeDef AS5600_get_angle(AS5600_TypeDef* const handle, uint16_t* const angle) {
	uint8_t data[2] = {0};
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, handle->i2c_timeout);
	*angle = ((data[0] << 8) | data[1]);
	return status;
}
HAL_StatusTypeDef AS5600_get_magnet_status(AS5600_TypeDef* const handle, uint8_t* const status) {
	return HAL_I2C_Mem_Read(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_STATUS, I2C_MEMADD_SIZE_8BIT, status, 1, handle->i2c_timeout);
}
HAL_StatusTypeDef AS5600_get_AGC_setting(AS5600_TypeDef* const handle, uint8_t* const agc) {
	return HAL_I2C_Mem_Read(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_AGC, I2C_MEMADD_SIZE_8BIT, agc, 1, handle->i2c_timeout);
}
HAL_StatusTypeDef AS5600_get_CORDIC_magnitude(AS5600_TypeDef* const handle, uint16_t* const magnitude) {
	uint8_t data[2] = {0};
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2c_handle, AS5600_SHIFTED_SLAVE_ADDRESS, AS5600_REGISTER_ANGLE_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, handle->i2c_timeout);
	*magnitude = ((data[0] << 8) | data[1]);
	return status;
}
