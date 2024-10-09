#include "mpu_6050.h"

const uint8_t mpu6050_dev_addr = 0x68 << 1;//0x68 << 1;
int gyroscope_output_rate = 8000; // 8khz

int write_to_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *reg_value){
	HAL_I2C_Mem_Write_IT(hi2c, mpu6050_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_value, 1);
	return 0;
}

int read_from_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *reg_value){
	HAL_I2C_Mem_Read_IT(hi2c, mpu6050_dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_value, 1);
	return 0;
}

int set_interrupt_enable(I2C_HandleTypeDef *hi2c, uint8_t interrupt_enable_value){
	write_to_register(hi2c, MPU6050_INT_EN_REG, &interrupt_enable_value);
	return 0;
}

int set_sampling_rate(I2C_HandleTypeDef *hi2c, uint8_t sampling_rate){

	uint8_t smplrt_div_value = (gyroscope_output_rate/sampling_rate) - 1;
	// set the smprlt to the register
	write_to_register(hi2c, MPU6050_SMPLRT_DIV_REG, &smplrt_div_value);

	return 0;
}

int clear_interrupt_status(I2C_HandleTypeDef *hi2c){
	write_to_register(hi2c, MPU6050_INT_STATUS_REG, 0b00000000);
	return 0;
}

int write_to_power_mgmt(I2C_HandleTypeDef *hi2c, uint8_t pwr_mgmt_reg_value){
	write_to_register(hi2c, MPU6050_POWER_MGMT_REG, pwr_mgmt_reg_value);
	return 0;
}
