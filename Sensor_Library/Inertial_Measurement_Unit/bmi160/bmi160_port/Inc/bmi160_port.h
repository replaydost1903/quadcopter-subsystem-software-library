#ifndef BMI160_PORTING_INC_BMI160_PORT_H_
#define BMI160_PORTING_INC_BMI160_PORT_H_

#include <stdio.h>
#include "main.h"
#include "stm32f4xx.h"
#include "bmi160.h"

#define BMI160_I2Cx	(&hi2c1)
extern I2C_HandleTypeDef hi2c1;

#define ACCEL_SENSITIVITY		( ( double )( 2048.0f ) )
#define GYRO_SENSITIVITY		( ( double )( 16.4f) )

#define BMI160_I2C_INTERFACE	(1U)			/*<! i2c interface communication, 1 - Enable; 0- Disable  <!*/
#define BMI160_SPI_INTERFACE	(0U)			/*<! spi interface communication, 1 - Enable; 0- Disable  <!*/

#define BMI160_DEV_ADDR			(0x68U)

extern void Error_Handler(void);

int8_t bmi160_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
int8_t bmi160_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void delay_ms(uint32_t period);

int8_t bmi160_interface_init(struct bmi160_dev *bmi160);
int8_t bmi160_calibration(struct bmi160_dev *bmi160,uint32_t IterTimeMS);
int8_t bmi160_get_acc_gyro(struct bmi160_dev *bmi160);

#endif /* BMI160_PORTING_INC_BMI160_PORT_H_ */

























