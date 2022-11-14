/// header guard
#ifndef AS5600_INCLUDED
#define AS5600_INCLUDED

/// includes
#include <stdint.h>
#include "stm32f4xx_hal.h"

/// AS5600 address
#define AS5600_SLAVE_ADDRESS			0x36
#define AS5600_SHIFTED_SLAVE_ADDRESS	0x6c
#define AS5600_I2C_TIMEOUT_DEFAULT		10	// ms
/// AS5600 registers
/* AS5600 configuration registers */
#define AS5600_REGISTER_ZMCO			0x00
#define AS5600_REGISTER_ZPOS_HIGH		0x01
#define AS5600_REGISTER_ZPOS_LOW		0x02
#define AS5600_REGISTER_MPOS_HIGH		0x03
#define AS5600_REGISTER_MPOS_LOW		0x04
#define AS5600_REGISTER_MANG_HIGH		0x05
#define AS5600_REGISTER_MANG_LOW		0x06
#define AS5600_REGISTER_CONF_HIGH		0x07
#define AS5600_REGISTER_CONF_LOW		0x08
/* AS5600 output registers */
#define AS5600_REGISTER_RAW_ANGLE_HIGH	0x0C
#define AS5600_REGISTER_RAW_ANGLE_LOW	0x0D
#define AS5600_REGISTER_ANGLE_HIGH		0x0E
#define AS5600_REGISTER_ANGLE_LOW		0x0F
/* AS5600 status registers */
#define AS5600_REGISTER_STATUS			0x0B
#define AS5600_REGISTER_AGC				0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH	0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW	0x1C
#define AS5600_REGISTER_BURN			0xFF

/// AS5600 settings
/* AS5600 power mode settings */
#define AS5600_POWER_MODE_NOM			1
#define AS5600_POWER_MODE_LPM1			2
#define AS5600_POWER_MODE_LPM2			3
#define AS5600_POWER_MODE_LPM3			4
#define AS5600_POWER_MODE_DEFAULT		AS5600_POWER_MODE_NOM
/* AS5600 hysteresis settings */
#define AS5600_HYSTERESIS_OFF			1
#define AS5600_HYSTERESIS_1LSB			2
#define AS5600_HYSTERESIS_2LSB			3
#define AS5600_HYSTERESIS_3LSB			4
#define AS5600_HYSTERESIS_DEFAULT		AS5600_HYSTERESIS_OFF
/* AS5600 output settings */
#define AS5600_OUTPUT_STAGE_FULL		1 /* ratiometric analog output ranging from GND-VCC*/
#define AS5600_OUTPUT_STAGE_REDUCED		2 /* ratiometric analog output ranging from 10% to 90% of VCC */
#define AS5600_OUTPUT_STAGE_PWM			3 /* Digital PWM output */
#define AS5600_OUTPUT_STAGE_DEFAULT		AS5600_OUTPUT_STAGE_FULL
/* AS5600 PWM frequency settings */
#define AS5600_PWM_FREQUENCY_115HZ		1
#define AS5600_PWM_FREQUENCY_230HZ		2
#define AS5600_PWM_FREQUENCY_460HZ		3
#define AS5600_PWM_FREQUENCY_920HZ		4
#define AS5600_PWM_FREQUENCY_DEFAULT	AS5600_PWM_FREQUENCY_115HZ
/* AS5600 slow filter settings */
#define AS5600_SLOW_FILTER_16X			1
#define AS5600_SLOW_FILTER_8X			2
#define AS5600_SLOW_FILTER_4X			3
#define AS5600_SLOW_FILTER_2X			4
#define AS5600_SLOW_FILTER_DEFAULT		AS5600_SLOW_FILTER_16X
/* AS5600 fast filter settings */
#define AS5600_FAST_FILTER_SLOW_ONLY	1
#define AS5600_FAST_FILTER_6LSB			2
#define AS5600_FAST_FILTER_7LSB			3
#define AS5600_FAST_FILTER_9LSB			4
#define AS5600_FAST_FILTER_18LSB		5
#define AS5600_FAST_FILTER_21LSB		6
#define AS5600_FAST_FILTER_24LSB		7
#define AS5600_FAST_FILTER_10LSB		8
#define AS5600_FAST_FILTER_DEFAULT		AS5600_FAST_FILTER_SLOW_ONLY
/* AS5600 watchdog settings */
#define AS5600_WATCHDOG_OFF				1
#define AS5600_WATCHDOG_ON				2
#define AS5600_WATCHDOG_DEFAULT			AS5600_WATCHDOG_ON

/// AS5600 definitions
/* AS5600 status definitions */
#define AS5600_AGC_MIN_GAIN_OVERFLOW	(uint8_t)(1UL << 3)		/* error bit indicates b-field is too strong */
#define AS5600_AGC_MAX_GAIN_OVERFLOW	(uint8_t)(1UL << 4)		/* error bit i5ndicates b-field is too weak */
#define AS5600_MAGNET_DETECTED			(uint8_t)(1UL << 5)		/* status bit indicates b-field is detected */
/* AS5600 direction definitions */
#define AS5600_DIR_CW					1
#define AS5600_DIR_CCW					2
/* AS5600 bit mask */
#define AS5600_12_BIT_MASK				(uint16_t)4095
/* AS5600 angle conversions */
#define AS5600_DEG_CONV 8.7890625e-2    /* 360/4096 */
#define AS5600_RAD_CONV 1.5339808e-3    /* 2pi/4096 */

/// AS5600 struct
typedef struct {	// variables are placed from largest to smallest to keep the struct as small as possible
    I2C_HandleTypeDef*	i2c_handle;
    GPIO_TypeDef*		dir_port;
    uint32_t			i2c_timeout;
    uint16_t			dir_pin;
    uint8_t				positive_rotation_direction;
    uint8_t				low_power_mode;
    uint8_t				hysteresis;
    uint8_t				output_mode;
    uint8_t				PWM_frequency;
    uint8_t				slow_filter;
    uint8_t				fast_filter_threshold;
    uint8_t				watchdog_timer;

    /* Private */
    volatile uint8_t	config_register[2];
} AS5600_TypeDef;

/// AS5600 functions
/* initialization */
AS5600_TypeDef*		AS5600_new								(void);
HAL_StatusTypeDef	AS5600_init								(AS5600_TypeDef* handle);
/* setters */
HAL_StatusTypeDef 	AS5600_set_start_position				(AS5600_TypeDef* const handle, const uint16_t position);
HAL_StatusTypeDef	AS5600_set_stop_position				(AS5600_TypeDef* const handle, const uint16_t position);
HAL_StatusTypeDef	AS5600_set_max_angle					(AS5600_TypeDef* const handle, const uint16_t angle);
HAL_StatusTypeDef	AS5600_set_positive_rotation_direction	(AS5600_TypeDef* const handle, const uint8_t direction);
HAL_StatusTypeDef	AS5600_set_low_power_mode				(AS5600_TypeDef* const handle, const uint8_t mode);
HAL_StatusTypeDef	AS5600_set_hysteresis					(AS5600_TypeDef* const handle, const uint8_t hysteresis);
HAL_StatusTypeDef	AS5600_set_output_mode					(AS5600_TypeDef* const handle, const uint8_t mode, uint8_t frequency);
HAL_StatusTypeDef	AS5600_set_slow_filter					(AS5600_TypeDef* const handle, const uint8_t mode);
HAL_StatusTypeDef	AS5600_set_fast_filter_threshold		(AS5600_TypeDef* const handle, const uint8_t threshold);
HAL_StatusTypeDef	AS5600_set_watchdog_timer				(AS5600_TypeDef* const handle, const uint8_t mode);
HAL_StatusTypeDef   AS5600_write_config_register            (AS5600_TypeDef* const handle);
/* getters */
HAL_StatusTypeDef	AS5600_get_rawAngle						(AS5600_TypeDef* const handle, uint16_t* const angle);
HAL_StatusTypeDef	AS5600_get_angle						(AS5600_TypeDef* const handle, uint16_t* const angle);
HAL_StatusTypeDef	AS5600_get_magnet_status				(AS5600_TypeDef* const handle, uint8_t* const status);
HAL_StatusTypeDef	AS5600_get_AGC_setting					(AS5600_TypeDef* const handle, uint8_t* const agc);
HAL_StatusTypeDef	AS5600_get_CORDIC_magnitude				(AS5600_TypeDef* const handle, uint16_t* const magnitude);

#endif	/* AS5600_INCLUDED */
