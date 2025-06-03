#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

/*
 *
 *	@brief: HMC5883L Settings for Quadcopter Primary Avionic Sub-System
 *
 *	@param: Operation Mode 			-->		Single Measurement Mode
 *	@param:	ODR						-->		160Hz (Max)
 *	@param:
 *
 *
 *
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>

/*
 * HMC5883L Device addresses according to I2C operating mode (R/W)
 */
#define HMC5883L_WRITE_ADDR					(0x3CU)
#define HMC5883L_READ_ADDR					(0x3DU)

/*
 * HMC5883L Register Map
 */
#define HMC5883L_CONFIG_A_REG				(0x00U)				/*! RW */
#define HMC5883L_CONFIG_B_REG				(0x01U)				/*! RW */
#define HMC5883L_MODE_REG					(0x02U)				/*! RW */
#define HMC5883L_DOR_X_MSB_REG				(0x03U)				/*! RO */
#define HMC5883L_DOR_X_LSB_REG				(0x04U)				/*! RO */
#define HMC5883L_DOR_Z_MSB_REG				(0x05U)				/*! RO */
#define HMC5883L_DOR_Z_LSB_REG				(0x06U)				/*! RO */
#define HMC5883L_DOR_Y_MSB_REG				(0x07U)				/*! RO */
#define HMC5883L_DOR_Y_LSB_REG				(0x08U)				/*! RO */
#define HMC5883L_STATUS_REG					(0x09U)				/*! RO */
#define HMC5883L_ID_A_REG					(0x0AU)				/*! RO */
#define HMC5883L_ID_B_REG					(0x0BU)				/*! RO */
#define HMC5883L_ID_C_REG					(0x0CU)				/*! RO */


/*
 * HMC5883L Some Important Register Address's Value
 */
#define HMC5883L_ID_A_REG_VAL				(0x48U)
#define HMC5883L_ID_B_REG_VAL				(0x34U)
#define HMC5883L_ID_C_REG_VAL				(0x33U)
#define HMC5883L_DRDY_BIT_MASK				(UINT8_C((0x1 << 0U)))

/*
 * HMC5883L OK State Flag
 */
#define HMC5883L_OK            				((int8_t)0)

/*
 * HMC5883L Error State Flag
 */
#define HMC5883L_DEV_NOT_FOUND 				((int8_t)-1)

/*
 * Function pointer type for reading data from a device register.
 *
 * Parameters:
 *   regaddr - The register address from which data will be read.
 *   pdata   - Pointer to the buffer where the read data will be stored.
 *   len     - The number of bytes to read.
 *
 * Returns:
 *   __int8_t - A status code (e.g., 0 for success, negative for error).
 */
typedef __int8_t(*read_fptr)(uint16_t regaddr,const uint8_t *pdata,uint16_t len);

/*
 * Function pointer type for writing data to a device register.
 *
 * Parameters:
 *   regaddr - The register address to which data will be written.
 *   pdata   - Pointer to the data buffer that contains the bytes to write.
 *   len     - The number of bytes to write.
 *
 * Returns:
 *   __int8_t - A status code (e.g., 0 for success, negative for error).
 */
typedef __int8_t(*write_fptr)(uint16_t regaddr,uint8_t *pdata,uint16_t len);

struct hmc5883l_data
{
	float mx;

	float my;

	float mz;
};

struct hmc5883l_interface
{
	I2C_HandleTypeDef *hi2c;

};


struct hmc5883l_dev
{
	struct hmc5883l_data mag_data;

	uint8_t dev_addr;

	uint8_t id_A;

	uint8_t id_B;

	uint8_t id_C;

	void *intfptr;

	read_fptr read_func;

	write_fptr write_func;

};

__int8_t hmc5883l_init(struct hmc5883l_dev *);

#endif /* INC_HMC5883L_H_ */



























