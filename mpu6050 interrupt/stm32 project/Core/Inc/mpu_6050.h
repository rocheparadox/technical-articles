// Author: Roche Christopher

#include<stdint.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_i2c.h"

#define MPU6050_CONFIG_REG 0x1A
#define MPU6050_GYRO_CONFIG_REG 0x1B
#define MPU6050_ACCEL_CONFIG_REG 0x1C
#define MPU6050_SMPLRT_DIV_REG 0x19
#define MPU6050_INT_EN_REG 0x38
#define MPU6050_INT_PIN_CFG_REG 0x37
#define MPU6050_INT_STATUS_REG 0x3A
#define MPU6050_POWER_MGMT_REG 0x6B

int write_to_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *reg_value);
int read_from_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *reg_value);
int set_configuration(I2C_HandleTypeDef hi2c, uint8_t config);
int set_gyroscope_configuration(I2C_HandleTypeDef hi2c, uint8_t gyro_config);
int set_sampling_rate(I2C_HandleTypeDef *hi2c, uint8_t sampling_rate);
int set_interrupt_enable(I2C_HandleTypeDef *hi2c, uint8_t interrupt_enable_value);
int write_to_power_mgmt(I2C_HandleTypeDef *hi2c, uint8_t pwr_mgmt_reg_value);
int clear_interrupt_status(I2C_HandleTypeDef *hi2c);


