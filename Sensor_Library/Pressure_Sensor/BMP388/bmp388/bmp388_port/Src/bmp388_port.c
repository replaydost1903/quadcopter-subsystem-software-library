#include "bmp388_port.h"

extern void Error_Handler(void);
extern struct bmp388_interface bmp388_intf;

static volatile uint32_t counter = 0U;

BMP3_INTF_RET_TYPE bmp388_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
	struct bmp388_interface *intf = (struct bmp388_interface *)intf_ptr;
	HAL_I2C_Mem_Read(intf->hi2c, (intf->dev_addr << 1U), reg_addr, 1U, read_data, len, HAL_MAX_DELAY);
	return ( BMP3_INTF_RET_SUCCESS );
}

BMP3_INTF_RET_TYPE bmp388_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,void *intf_ptr)
{
	struct bmp388_interface *intf = (struct bmp388_interface *)intf_ptr;
	HAL_I2C_Mem_Write(intf->hi2c, ((intf->dev_addr << 1U) | 0x1U), reg_addr, 1U, (uint8_t*)read_data, len, HAL_MAX_DELAY);
	return ( BMP3_INTF_RET_SUCCESS );
}

void delay_us(uint32_t period, void *intf_ptr)
{
	struct bmp388_interface *intf = (struct bmp388_interface *)intf_ptr;

	intf_ptr = (struct bmp388_interface *)intf_ptr;

	HAL_TIM_Base_Start_IT(intf->htim);

	while(counter != period);

	HAL_TIM_Base_Stop_IT(intf->htim);

	counter = 0U;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ( htim == NULL )
	{
		Error_Handler();
	}

	if ( htim->Instance == (TIM_TypeDef*)(bmp388_intf.htim->Instance) )
	{
		++counter;
	}

}

BMP3_INTF_RET_TYPE bmp388_interface_init(struct bmp3_dev *bmp388,struct bmp388_interface *bmp388_intf)
{
	if( ( bmp388 == NULL ) || ( bmp388_intf == NULL ) )
	{
		Error_Handler();
	}

	struct bmp3_settings settings = { 0 };
	int8_t rslt = 0U;
	uint16_t settings_sel = 0U;

	bmp388->intf = BMP3_I2C_INTF;
	bmp388->read = bmp388_read;
	bmp388->write = bmp388_write;
	bmp388->delay_us = delay_us;
	bmp388->intf_ptr = bmp388_intf;

	rslt = bmp3_selftest_check(bmp388);

	if ( rslt !=  BMP3_OK )
	{
		printf("BMP388 self-test failure !\n");
		Error_Handler();
	}

	printf("BMP388 self-test success !\n");

	rslt = bmp3_init(bmp388);

	if ( rslt !=  BMP3_OK )
	{
		printf("BMP388 initialization failure !\n");
		Error_Handler();
	}

	printf("BMP388 initialization success !\n");
	printf("Chip ID 0x%X\n" , bmp388->chip_id);

	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	/*
	 * @brief
	 * Recommended filter settings based on use cases for Drone.
	 * Standart Resolution for Oversampling Settings.
	 * @params
	 *		- OVERSAMPLING COEFF. FOR PRESSURE 			- x8		- BMP3_OVERSAMPLING_8X
	 *		- OVERSAMPLING COEFF. FOR TEMPERATURE 		- x1		- BMP3_NO_OVERSAMPLING
	 *		- IIR FILTER COEFFICIENT 3					- 2			- BMP3_IIR_FILTER_COEFF_3
	 *		- ODR 										- 50Hz		- BMP3_ODR_50_HZ
	 *		- RMS NOISE[cm]								- 11
	 */

	settings.odr_filter.iir_filter 		= 	BMP3_IIR_FILTER_COEFF_31;
	settings.odr_filter.press_os 		= 	BMP3_OVERSAMPLING_8X;
	settings.odr_filter.temp_os 		= 	BMP3_NO_OVERSAMPLING;
	settings.odr_filter.odr 			= 	BMP3_ODR_50_HZ;
	settings.int_settings.drdy_en 		= 	BMP3_SEL_DRDY_EN;
	settings.int_settings.latch 		= 	BMP3_INT_PIN_LATCH;
	settings.int_settings.level 		= 	BMP3_INT_PIN_ACTIVE_LOW;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
				 BMP3_SEL_DRDY_EN | BMP3_SEL_IIR_FILTER | BMP3_SEL_LEVEL;

	rslt = bmp3_set_sensor_settings(settings_sel, &settings, bmp388);

	if ( rslt !=  BMP3_OK )
	{
		printf("BMP388 set sensor settings failure !\n");
		Error_Handler();
	}
	uint8_t data = (0x1<<6U)|(0x1<<2U);
	uint8_t addr = BMP3_REG_INT_CTRL;
	bmp3_set_regs(&addr,(const uint8_t*)&data,1U,bmp388);

	printf("BMP388 set sensor settings success !\n");

	settings.op_mode = BMP3_MODE_NORMAL;
	rslt = bmp3_set_op_mode(&settings, bmp388);

	if ( rslt !=  BMP3_OK )
	{
		printf("BMP388 sensor op mode failure !\n");
		Error_Handler();
	}

	printf("BMP388 sensor op mode success !\n");

	HAL_Delay(3000);

	return ( BMP3_INTF_RET_SUCCESS );

}

BMP3_INTF_RET_TYPE get_bmp388_sensor_data(struct bmp3_dev *bmp388)
{
	if( ( bmp388 == NULL ) )
	{
		Error_Handler();
	}

	struct bmp3_status status = { { 0 } };
	int8_t rslt = 0U;

	rslt = bmp3_get_status(&status, bmp388);

	if ( rslt !=  BMP3_OK )
	{
		  printf("BMP388 sensor get status failure !\n");
		  Error_Handler();
	}

	/* Read temperature and pressure data iteratively based on data ready interrupt */
	if ( (rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE) )
	{
		  /*
		   * First parameter indicates the type of data to be read
		   * BMP3_PRESS_TEMP : To read pressure and temperature data
		   * BMP3_TEMP       : To read only temperature data
		   * BMP3_PRESS      : To read only pressure data
		   */
		  rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp388->pressure_data , bmp388);

		  if ( rslt !=  BMP3_OK )
		  {
			  printf("BMP388 sensor get status failure !\n");
			  Error_Handler();
		  }

		  /* NOTE : Read status register again to clear data ready interrupt status */
		  rslt = bmp3_get_status(&status, bmp388);

		  if ( rslt !=  BMP3_OK )
		  {
			  printf("BMP388 sensor get status failure !\n");
			  Error_Handler();
		  }
	}

	return ( BMP3_INTF_RET_SUCCESS );
}

BMP3_INTF_RET_TYPE bmp388_calibration(struct bmp3_dev *bmp388,uint32_t sample_count)
{
	if( ( bmp388 == NULL ) || ( sample_count <= 0U ) )
	{
		Error_Handler();
	}

	/*!< Pressure Calibration >!*/
	uint32_t iter = 0U;
	double calibration_data = 0.0f;

	while ( iter++ < sample_count )
	{
		if ( get_bmp388_sensor_data(bmp388) != BMP3_OK )
		{
			  printf("BMP388 sensor get data failure !\n");
			  Error_Handler();
		}

		calibration_data += bmp388->pressure_data.pressure;
	}

	calibration_data /= (double)iter;

	bmp388->pressure_data.ground_pressure = calibration_data;

	/*!< Relativity Altitude Offset Calculation >!*/
	bmp388->pressure_data.relative_offset = 0.0f;
	iter = 0.0f;

	while ( iter++ < sample_count )
	{
		if ( get_bmp388_sensor_data(bmp388) != BMP3_OK )
		{
			  printf("BMP388 sensor get data failure !\n");
			  Error_Handler();
		}

		bmp388->pressure_data.relative_offset += -((bmp388->pressure_data.pressure-bmp388->pressure_data.ground_pressure) / (DENSITY_AIR_KG_M3 * GRAVITY_AIR_KG_MPS2));
	}

	bmp388->pressure_data.relative_offset /= (double)iter;

	return ( BMP3_INTF_RET_SUCCESS );
}


BMP3_INTF_RET_TYPE bmp388_get_altitude(struct bmp3_dev *bmp388)
{
	if ( bmp388 == NULL )
	{
		Error_Handler();
	}

	/*!< Compute altitude (relative to initial level) >!*/
	bmp388->pressure_data.relative_altitude = (-(bmp388->pressure_data.pressure - bmp388->pressure_data.ground_pressure) /\
									 (DENSITY_AIR_KG_M3 * GRAVITY_AIR_KG_MPS2) - bmp388->pressure_data.relative_offset );

	return ( BMP3_INTF_RET_SUCCESS );
}







