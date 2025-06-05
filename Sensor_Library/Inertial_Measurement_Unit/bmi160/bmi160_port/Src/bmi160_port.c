#include "bmi160_port.h"
#include "bmi160_defs.h"

__inline static void IsDeviceReady(uint8_t dev_addr);

__inline static void IsDeviceReady(uint8_t dev_addr)
{
	  if(HAL_OK == HAL_I2C_IsDeviceReady(BMI160_I2Cx, (dev_addr << 1U), 100, HAL_MAX_DELAY))
	  {
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1U);
	  }
	  else
	  {
		  Error_Handler();
	  }
}

int8_t bmi160_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
	dev_addr = ((dev_addr << 1U) | 0x0U);
	HAL_I2C_Mem_Write(BMI160_I2Cx, dev_addr, reg_addr, 1U, read_data, len, HAL_MAX_DELAY);
	return ( BMI160_OK );
}
int8_t bmi160_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	dev_addr = ((dev_addr << 1U) | 0x1U);
	HAL_I2C_Mem_Read(BMI160_I2Cx, dev_addr, reg_addr, 1U, data, len, HAL_MAX_DELAY);
	return ( BMI160_OK );
}
void delay_ms(uint32_t period)
{
	HAL_Delay(period);		//Systick yerine TIM6 olacak
}

int8_t bmi160_interface_init(struct bmi160_dev *bmi160)
{
	/*<! Check null-pointer  <!*/
	if( bmi160 == NULL )
	{
	  Error_Handler();
	}

#if ( ( BMI160_I2C_INTERFACE) && ( !BMI160_SPI_INTERFACE ) )

	int8_t rslt = 0U;

	bmi160_soft_reset(bmi160);

	bmi160->id = BMI160_DEV_ADDR;
	bmi160->intf = BMI160_I2C_INTF;
	bmi160->read = bmi160_i2c_read;
	bmi160->write = bmi160_i2c_write;
	bmi160->delay_ms = delay_ms;

	/* After sensor init introduce 200 msec sleep */
	bmi160->delay_ms(200);

	rslt = bmi160_init(bmi160);

	if ( rslt != BMI160_OK )
	{
		printf("BMI160 initialization failure !\n");
		Error_Handler();
	}

	printf("BMI160 initialization success !\n");
	printf("Chip ID 0x%X\n" , bmi160->chip_id);

//	//acceleration self-test configuration
//	rslt = bmi160_perform_self_test(BMI160_ACCEL_ONLY,bmi160);
//
//	if( rslt != BMI160_OK )
//	{
//		printf("BMI160 acceleration self-test failure !\n");
//		Error_Handler();
//	}
//
//	printf("BMI160 acceleration self-test success !\n");
//
//	//gyroscope self-test configuration
//	rslt = bmi160_perform_self_test(BMI160_GYRO_ONLY,bmi160);
//
//	if( rslt != BMI160_OK )
//	{
//		printf("BMI160 gyroscope self-test failure !\n");
//		Error_Handler();
//	}
//
//	printf("BMI160 gyroscope self-test success !\n");

	/* Select the Output data rate, range of accelerometer sensor */
	bmi160->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	bmi160->accel_cfg.range = BMI160_ACCEL_RANGE_16G;
	bmi160->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

	/* Select the power mode of accelerometer sensor */
	bmi160->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	/* Select the Output data rate, range of Gyroscope sensor */
	bmi160->gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	bmi160->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	bmi160->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

	/* Select the power mode of Gyroscope sensor */
	bmi160->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(bmi160);

	if( rslt != BMI160_OK )
	{
		printf("BMI160 sensor configuration failure !\n");
		Error_Handler();
	}

	printf("BMI160 sensor configuration success !\n");

	/*
	 * Data Ready Interrupt Configurations
	 */
	struct bmi160_int_settg int_config = {0};

	int_config.int_channel 	= 	BMI160_INT_CHANNEL_BOTH;
	int_config.int_type 	= 	BMI160_ACC_GYRO_DATA_RDY_INT;
	int_config.int_pin_settg.output_en 		= 		1U;														//Output enabled
	int_config.int_pin_settg.output_mode 	=		0U;														//Push-pull
	int_config.int_pin_settg.output_type 	=		1U;														//Active low
	int_config.int_pin_settg.edge_ctrl		=		1U;														//Edge Trigger
	int_config.int_pin_settg.input_en 		=		0U;														//Input disabled
	int_config.int_pin_settg.latch_dur 		=		BMI160_LATCH_DUR_NONE;									//Latched Mode

	if ( bmi160_set_int_config(&int_config, bmi160) != BMI160_OK )
	{
		printf("BMI160 data ready interrupt configuration is failure !\n");
		Error_Handler();
	}

	/* After sensor init introduce 200 msec sleep */
	HAL_Delay(200);

	return ( BMI160_OK );

	#elif ( ( BMI160_SPI_INTERFACE) && ( !BMI160_I2C_INTERFACE ) )


	#else
		Error_Handler();
	#endif
	return ( BMI160_E_INVALID_CONFIG );
}

int8_t bmi160_calibration(struct bmi160_dev *bmi160,uint32_t IterTimeMS)
{
	/*<! Check null-pointer  <!*/
	if( bmi160 == NULL )
	{
	   Error_Handler();
	}

	/*<! Check iteration time  <!*/
	if ( IterTimeMS <= 0)
	{
	   Error_Handler();
	}

	/*<! Reset accel data offset value  <!*/
	bmi160->accel_data.x_offset = 0.0f;
	bmi160->accel_data.y_offset = 0.0f;
	bmi160->accel_data.z_offset = 0.0f;

	/*<! Reset gyro data offset value  <!*/
	bmi160->gyro_data.x_offset = 0.0f;
	bmi160->gyro_data.y_offset = 0.0f;
	bmi160->gyro_data.z_offset = 0.0f;

	uint32_t sample_num = 0U;
	uint32_t current_tick = HAL_GetTick();

	/*<! Get data for the duration of the IterTimeMS
	 * During this process, the sensor must be stationary and in a flat plane <!*/
	while( ( HAL_GetTick() - current_tick ) < IterTimeMS )
	{
	  if( bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),&bmi160->accel_data,&bmi160->gyro_data,bmi160) != BMI160_OK)
	  {
		  printf("BMI160 sensor data failed !\n");
		  Error_Handler();
	  }

	  bmi160->accel_data.x_offset += (double)bmi160->accel_data.x;
	  bmi160->accel_data.y_offset += (double)bmi160->accel_data.y;
	  bmi160->accel_data.z_offset += (double)bmi160->accel_data.z;

	  bmi160->gyro_data.x_offset  += (double)bmi160->gyro_data.x;
	  bmi160->gyro_data.y_offset  += (double)bmi160->gyro_data.y;
	  bmi160->gyro_data.z_offset  += (double)bmi160->gyro_data.z;

	  ++sample_num;
	}

	/*<! 3-axis accelerometer calibration result offset value  <!*/
	bmi160->accel_data.x_offset = (((bmi160->accel_data.x_offset / (double)sample_num))/ACCEL_SENSITIVITY);
	bmi160->accel_data.y_offset = (((bmi160->accel_data.y_offset / (double)sample_num))/ACCEL_SENSITIVITY);
	bmi160->accel_data.z_offset = (((bmi160->accel_data.z_offset / (double)sample_num))/ACCEL_SENSITIVITY - 9.81f);

	/*<! 3-axis gyroscope calibration result offset value  <!*/
	bmi160->gyro_data.x_offset = ((bmi160->gyro_data.x_offset / (double)sample_num))/GYRO_SENSITIVITY;
	bmi160->gyro_data.y_offset = ((bmi160->gyro_data.y_offset / (double)sample_num))/GYRO_SENSITIVITY;
	bmi160->gyro_data.z_offset = ((bmi160->gyro_data.z_offset / (double)sample_num))/GYRO_SENSITIVITY;

	return ( BMI160_OK );
}

int8_t bmi160_get_acc_gyro(struct bmi160_dev *bmi160)
{
	if ( bmi160 == NULL )
	{
		Error_Handler();
	}

    if( bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),&bmi160->accel_data,&bmi160->gyro_data,bmi160) != BMI160_OK)
    {
	    printf("BMI160 sensor data failed !\n");
	    Error_Handler();
    }

    bmi160->accel_data.xd = (((double)bmi160->accel_data.x)/ACCEL_SENSITIVITY - bmi160->accel_data.x_offset);
    bmi160->accel_data.yd = (((double)bmi160->accel_data.y)/ACCEL_SENSITIVITY - bmi160->accel_data.y_offset);
    bmi160->accel_data.zd = (((double)bmi160->accel_data.z)/ACCEL_SENSITIVITY - bmi160->accel_data.z_offset);

    bmi160->gyro_data.xd = (((double)bmi160->gyro_data.x)/GYRO_SENSITIVITY - bmi160->gyro_data.x_offset);
    bmi160->gyro_data.yd = (((double)bmi160->gyro_data.y)/GYRO_SENSITIVITY - bmi160->gyro_data.y_offset);
    bmi160->gyro_data.zd = (((double)bmi160->gyro_data.z)/GYRO_SENSITIVITY - bmi160->gyro_data.z_offset);

	return ( BMI160_OK );
}


































