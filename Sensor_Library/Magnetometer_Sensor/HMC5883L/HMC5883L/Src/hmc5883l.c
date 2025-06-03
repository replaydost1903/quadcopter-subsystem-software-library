#include "hmc5883l.h"
#include <stdio.h>

#define MAX_I2C_DEV_NUM		(127U)

extern void Error_Handler(void);

extern struct hmc5883l_interface hmc5883l_intf;

/*
 * Helper Function Declarations
 */
static __int8_t hmc5883l_self_test(struct hmc5883l_dev *hmc5883l);
static __int8_t hmc5883l_is_ready(struct hmc5883l_dev*,struct hmc5883l_interface *hmc5883l_intf);
static __int8_t hmc5883l_read(uint16_t regaddr,const uint8_t *pdata,uint16_t len);
static __int8_t hmc5883l_write(uint16_t regaddr,uint8_t *pdata,uint16_t len);


static __int8_t hmc5883l_read(uint16_t regaddr,const uint8_t *pdata,uint16_t len)
{
	HAL_StatusTypeDef status = 0U;

	if ( pdata == NULL )
	{
		printf("Null pointer error !\n");
		Error_Handler();
	}

	status = HAL_I2C_Mem_Read(hmc5883l_intf.hi2c, HMC5883L_READ_ADDR,regaddr,1,(uint8_t*)pdata,len,HAL_MAX_DELAY);

	if ( status != HAL_OK )
	{
		printf("I2C Protocol Error !\n");
		Error_Handler();
	}

	return ( HMC5883L_OK );
}
static __int8_t hmc5883l_write(uint16_t regaddr,uint8_t *pdata,uint16_t len)
{
	HAL_StatusTypeDef status = 0U;

	if ( pdata == NULL )
	{
		printf("Null pointer error !\n");
		Error_Handler();
	}

	status = HAL_I2C_Mem_Write(hmc5883l_intf.hi2c, HMC5883L_WRITE_ADDR,regaddr,1,pdata,len,HAL_MAX_DELAY);

	if ( status != HAL_OK )
	{
		printf("I2C Protocol Error !\n");
		Error_Handler();
	}

	return ( HMC5883L_OK );
}
__int8_t hmc5883l_init(struct hmc5883l_dev *hmc5883l)
{

	hmc5883l->intfptr = &hmc5883l_intf;
	hmc5883l->read_func = hmc5883l_read;
	hmc5883l->write_func = hmc5883l_write;

	uint8_t read_data[3] = {0};
	uint8_t temp = 0U;

	if ( hmc5883l == NULL )
	{
		printf("Null pointer error !\n");
		Error_Handler();
	}

	if ( hmc5883l_is_ready(hmc5883l,&hmc5883l_intf) != HMC5883L_OK)
	{
		printf("Device Not Founded !\n");
		Error_Handler();
	}

	if ( hmc5883l->read_func(HMC5883L_ID_A_REG,(uint8_t*)read_data,3U) != HMC5883L_OK)
	{
		printf("I2C Read Error!\n");
		Error_Handler();
	}

	hmc5883l->id_A = *read_data;

	hmc5883l->id_B = *(read_data + 1);

	hmc5883l->id_C = *(read_data + 2);

	temp = (hmc5883l->id_A == HMC5883L_ID_A_REG_VAL) && (hmc5883l->id_B == HMC5883L_ID_B_REG_VAL) &&\
			(hmc5883l->id_C == HMC5883L_ID_C_REG_VAL);

	if( !temp )
	{
		printf("All ID Register Read Error!\n");
		Error_Handler();
	}

	if ( hmc5883l_self_test(hmc5883l) != HMC5883L_OK)
	{
		printf("HMC5883L self-test error !\n");
		Error_Handler();
	}

	//Single Measurement Mode Configuration

	uint8_t tx_data[1] = {0};
	// Write CRA (00) – send 0x3C 0x00 0x70 (8-average, 15 Hz default or any other rate, normal measurement)
	tx_data[0] = 0x70;
	hmc5883l->write_func(HMC5883L_CONFIG_A_REG,tx_data,1U);

	// Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5, or any other desired gain)
	tx_data[0] = 0xA0;
	hmc5883l->write_func(HMC5883L_CONFIG_B_REG,tx_data,1U);

	// Write Mode (02) – send 0x3C 0x02 0x01 (Single-measurement mode)
	tx_data[0] = 0x01;
	hmc5883l->write_func(HMC5883L_MODE_REG,tx_data,1U);

	return ( HMC5883L_OK );
}
static __int8_t hmc5883l_is_ready(struct hmc5883l_dev *hmc5883l,struct hmc5883l_interface *hmc5883l_intf)
{
	HAL_StatusTypeDef status = 0U;
	uint8_t dev_no = 0U;

	if ( ( hmc5883l == NULL ) || ( hmc5883l_intf == NULL ) )
	{
		printf("Null pointer error !\n");
		Error_Handler();
	}

	while ( dev_no++ < MAX_I2C_DEV_NUM )
	{

		status = HAL_I2C_IsDeviceReady(hmc5883l_intf->hi2c,(uint16_t)(dev_no << 1U),100,HAL_MAX_DELAY);

		if ( status == HAL_OK)
		{
			hmc5883l->dev_addr = (uint8_t)dev_no;

			printf("Device found address 0x%X\n",hmc5883l->dev_addr);

			return ( HMC5883L_OK );
		}

	}

	return ( HMC5883L_DEV_NOT_FOUND );
}
static __int8_t hmc5883l_self_test(struct hmc5883l_dev *hmc5883l)
{
	if ( hmc5883l == NULL )
	{
		printf("Null pointer error !\n");
		Error_Handler();
	}

	uint8_t temp = 0U;
	uint8_t rx_data[6] = {0};
	uint8_t tx_data[1] = {0};

	uint16_t self_test_x = 0,self_test_y = 0,self_test_z = 0;

	// Write CRA (00) – send 0x3C 0x00 0x71 (8-average, 15 Hz default, positive self test measurement)
	tx_data[0] = 0x71;
	hmc5883l->write_func(HMC5883L_CONFIG_A_REG,tx_data,1U);

	// Write CRB (01) – send 0x3C 0x01 0xA0 (Gain=5)
	tx_data[0] = 0xA0;
	hmc5883l->write_func(HMC5883L_CONFIG_B_REG,tx_data,1U);

	// Write Mode (02) – send 0x3C 0x02 0x00 (Continuous-measurement mode)
	tx_data[0] = 0x00;
	hmc5883l->write_func(HMC5883L_MODE_REG,tx_data,1U);

	// Wait DRDY hardware interrupt pin
	while( !( hmc5883l->read_func(HMC5883L_STATUS_REG,rx_data,1U) ) )
	{
		if ( ( rx_data[0] & HMC5883L_DRDY_BIT_MASK ) == HMC5883L_DRDY_BIT_MASK )
		{
			break;
		}
	}

	// Read 6 bytes
	hmc5883l->read_func(HMC5883L_DOR_X_MSB_REG,rx_data,6U);

	self_test_x = (uint16_t)(( rx_data[0] << 8U ) | rx_data[1]);
	self_test_z = (uint16_t)(( rx_data[2] << 8U ) | rx_data[3]);
	self_test_y = (uint16_t)(( rx_data[4] << 8U ) | rx_data[5]);

	temp = (self_test_x >= 243) &&  (self_test_x <= 575) &&\
		   (self_test_y >= 243) &&  (self_test_y <= 575) &&\
		   (self_test_z >= 243) &&  (self_test_z <= 575);

	if ( !temp )
	{
		printf("HMC5883C self-test error !\n");
		Error_Handler();
	}

	printf("HMC5883C self-test successful !\n");

	// All 3 axes pass positive self test and Exit self test mode and this procedure)
	tx_data[0] = 0x70;
	hmc5883l->write_func(HMC5883L_CONFIG_A_REG,tx_data,1U);

	return ( HMC5883L_OK );
}
