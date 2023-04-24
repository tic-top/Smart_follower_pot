#include "soil.h"

extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;
static const uint8_t TMP102_ADDR = 0x48 << 1; // Use 8-bit address
static const uint8_t REG_TEMP = 0x00;

uint32_t soil_moisture(){
	uint32_t ADC_VAL = 0;
	uint32_t buf[3] = { 0 };
	HAL_ADC_Start_DMA(&hadc1, buf, 3);
	HAL_Delay(1);
	ADC_VAL = buf[0];
	return ADC_VAL;
}


void pump(uint32_t moisture){
	//FIXME: moisture_map
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_Delay(moisture);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}


float temperature_sensor(){
	HAL_StatusTypeDef ret;
	uint8_t buf[12];
	int16_t val;
	float temp_c;
    // Tell TMP102 that we want to read from the temperature register
    buf[0] = REG_TEMP;
    ret = HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
    	return -1;
    } else {

      // Read 2 bytes from the temperature register
      ret = HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, buf, 2, HAL_MAX_DELAY);
      if ( ret != HAL_OK ) {
//        strcpy((char*)buf, "Error Rx\r\n");
    	  return -1;
      } else {

        //Combine the bytes
        val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);

        // Convert to 2's complement, since temperature can be negative
        if ( val > 0x7FF ) {
          val |= 0xF000;
        }

        // Convert to float temperature value (Celsius)
        temp_c = val * 0.0625;

        // Convert temperature to decimal format
//        temp_c *= 100;
//        sprintf((char*)buf,
//              "%u.%u C\r\n",
//              ((unsigned int)temp_c / 100),
//              ((unsigned int)temp_c % 100));
      }
    }

    // Send out buffer (temperature or error message)
    return temp_c;
//    HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

    // Wait
//    HAL_Delay(500);

}
