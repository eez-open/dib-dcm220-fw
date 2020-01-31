#include "main.h"

#include <math.h>

////////////////////////////////////////////////////////////////////////////////

#define FIRMWARE_VERSION_MAJOR 0x00
#define FIRMWARE_VERSION_MINOR 0x04

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern SDADC_HandleTypeDef hsdadc1;
extern CRC_HandleTypeDef hcrc;

////////////////////////////////////////////////////////////////////////////////

#define REG0_OE1_MASK     (1 << 0)
#define REG0_CC1_MASK     (1 << 1)
#define REG0_OE2_MASK     (1 << 2)
#define REG0_CC2_MASK     (1 << 3)
#define REG0_PWRGOOD_MASK (1 << 4)

////////////////////////////////////////////////////////////////////////////////

#define BUFFER_SIZE 20

uint8_t output[BUFFER_SIZE];
uint8_t input[BUFFER_SIZE];

uint16_t *output_uMon0 = (uint16_t *)(output + 2);
uint16_t *output_iMon0 = (uint16_t *)(output + 4);
uint16_t *output_uMon1 = (uint16_t *)(output + 6);
uint16_t *output_iMon1 = (uint16_t *)(output + 8);

uint16_t *output_temperature0 = (uint16_t *)(output + 10);
uint16_t *output_temperature1 = (uint16_t *)(output + 12);
uint16_t *output_temperature2 = (uint16_t *)(output + 14);
uint32_t *output_CRC = (uint32_t *)(output + BUFFER_SIZE - 4);

uint16_t *inputSetValues = (uint16_t *)(input + 2);

int transferCompleted;

uint16_t uMon[2];
uint16_t iMon[2];

uint16_t temperature[3] = { 65535, 65535, 65535 }; // 65535 = invalid (not measured yet) temperature

uint16_t uSet[2];
uint16_t iSet[2];

uint16_t uSetNext[2];
uint16_t iSetNext[2];

int updateSetValues[2];

int loopOperationIndex;

////////////////////////////////////////////////////////////////////////////////

int isInterrupt() {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t reg0;

void outputEnable(int iChannel, int enable) {
	uint8_t mask = iChannel == 0 ? REG0_OE1_MASK : REG0_OE2_MASK;

	if (enable) {
		if (reg0 & mask) {
			// already enabled
			return;
		}
		reg0 |= mask;
	} else {
		if (!(reg0 & mask)) {
			// already disabled
			return;
		}
		reg0 &= ~mask;
	}

	if (iChannel == 0) {
		HAL_GPIO_WritePin(OE_1_GPIO_Port, OE_1_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(OE_2_GPIO_Port, OE_2_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	// make sure we update SET values for this channel
	updateSetValues[iChannel] = 1;
	loopOperationIndex = 0;
}

void ccLED(int iChannel, int enable) {
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (iChannel == 0) {
        HAL_GPIO_WritePin(CC_LED_1_GPIO_Port, CC_LED_1_Pin, state);
    } else {
        HAL_GPIO_WritePin(CC_LED_2_GPIO_Port, CC_LED_2_Pin, state);
    }
}

void readPwrGood() {
    if (HAL_GPIO_ReadPin(PWRGOOD_GPIO_Port, PWRGOOD_Pin) == GPIO_PIN_SET) {
        reg0 |= REG0_PWRGOOD_MASK;
    } else {
        outputEnable(0, 0);
        outputEnable(1, 0);
        reg0 &= ~REG0_PWRGOOD_MASK;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == PWRGOOD_Pin) {
        readPwrGood();
    } else if (GPIO_Pin == CC_1_Pin) {
        if (HAL_GPIO_ReadPin(CC_1_GPIO_Port, CC_1_Pin)) {
            reg0 |= REG0_CC1_MASK;
        } else {
            reg0 &= ~REG0_CC1_MASK;
        }

        // update CC LED
        HAL_GPIO_WritePin(CC_LED_1_GPIO_Port, CC_LED_1_Pin, (reg0 & REG0_OE1_MASK) && (reg0 & REG0_CC1_MASK));
    } else if (GPIO_Pin == CC_2_Pin) {
        if (HAL_GPIO_ReadPin(CC_2_GPIO_Port, CC_2_Pin)) {
            reg0 |= REG0_CC2_MASK;
        } else {
            reg0 &= ~REG0_CC2_MASK;
        }

        // update CC LED
        HAL_GPIO_WritePin(CC_LED_2_GPIO_Port, CC_LED_2_Pin, (reg0 & REG0_OE2_MASK) && (reg0 & REG0_CC2_MASK));
    } else if (GPIO_Pin == DIB_SYNC_Pin) {
    }
}

////////////////////////////////////////////////////////////////////////////////

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

void dacSetVoltage(uint8_t iChannel, uint16_t value) {
	if (iChannel == 0) {
		HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095 - value);
	} else {
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095 - value);
	}
}

void dacSetCurrent(uint8_t iChannel, uint16_t value) {
	if (iChannel == 0) {
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, (uint32_t)value * 1440 / 4095);
	} else {
		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, (uint32_t)value * 1440 / 4095);
	}
}


void dacInit() {
	dacSetVoltage(0, 0); // U_SET#1 <- 0

	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	dacSetCurrent(0, 0); // I_SET#1 <- 0

	dacSetVoltage(1, 0); // U_SET#2 <- 0

	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	dacSetCurrent(1, 0); // I_SET#2 <- 0
}

////////////////////////////////////////////////////////////////////////////////

void sdadcInit() {
    // 6: U_MON CH1
    if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_6, SDADC_CONF_INDEX_0) != HAL_OK) {
        Error_Handler();
    }

    // 5: I_MON CH1
    if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_5, SDADC_CONF_INDEX_1) != HAL_OK) {
        Error_Handler();
    }

    // 4: U_MON CH2
    if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_4, SDADC_CONF_INDEX_0) != HAL_OK) {
        Error_Handler();
    }

    // 8: I_MON CH2
    if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_8, SDADC_CONF_INDEX_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER) != HAL_OK) {
        Error_Handler();
    }
}

uint16_t sdadcRead(uint32_t channel) {
    if (HAL_SDADC_ConfigChannel(&hsdadc1, channel, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_SDADC_Start(&hsdadc1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_SDADC_PollForConversion(&hsdadc1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();
    }

    uint16_t value = (uint16_t)(HAL_SDADC_GetValue(&hsdadc1) + 32768);

    if (HAL_SDADC_Stop(&hsdadc1) != HAL_OK) {
        Error_Handler();
    }

    return (uint16_t)value;
}

uint16_t sdadcReadVoltage(int iChannel) {
    return sdadcRead(iChannel == 0 ? SDADC_CHANNEL_6 : SDADC_CHANNEL_4);
}

uint16_t sdadcReadCurrent(int iChannel) {
    return sdadcRead(iChannel == 0 ? SDADC_CHANNEL_5 : SDADC_CHANNEL_8);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t adcConvertedValues[3];
uint32_t lastAdcStartTickCounter = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	temperature[0] = (uint16_t)adcConvertedValues[0];
	temperature[1] = (uint16_t)adcConvertedValues[1];
	temperature[2] = (uint16_t)adcConvertedValues[2];
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
}

void adcLoop() {
	uint32_t tickCounter = HAL_GetTick();
	int32_t diff = tickCounter - lastAdcStartTickCounter;
	if (lastAdcStartTickCounter == 0 || diff > 1000) {
		HAL_ADC_Start_DMA(&hadc1, adcConvertedValues, 3);
		lastAdcStartTickCounter = tickCounter;
	}
}

////////////////////////////////////////////////////////////////////////////////

#define SPI_SLAVE_SYNBYTE         0x53
#define SPI_MASTER_SYNBYTE        0xAC

void slaveSynchro(void) {
    uint8_t txBuffer[3] = { SPI_SLAVE_SYNBYTE, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR };
    uint8_t rxBuffer[3] = { 0, 0, 0 };
    do {
        if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&txBuffer, (uint8_t *)&rxBuffer, 3, HAL_MAX_DELAY) != HAL_OK) {
            Error_Handler();
        }
    } while (rxBuffer[0] != SPI_MASTER_SYNBYTE);
}

////////////////////////////////////////////////////////////////////////////////

// code U_SET on CH1
void loopOperation_DacCodeU0() {
    if (uSet[0] != uSetNext[0] || updateSetValues[0]) {
    	if (reg0 & REG0_OE1_MASK) {
    		uSet[0] = uSetNext[0];
    		dacSetVoltage(0, uSet[0]);
    	} else {
    		// set voltage to zero if channel is turned off
    		dacSetVoltage(0, 0);
    	}
    }
}

// code I_SET on CH1
void loopOperation_DacCodeI0() {
    if (iSet[0] != iSetNext[0] || updateSetValues[0]) {
    	if (reg0 & REG0_OE1_MASK) {
    		iSet[0] = iSetNext[0];
    		dacSetCurrent(0, iSet[0]);
    	} else {
    		// set current to zero if channel is turned off
    		dacSetCurrent(0, 0);
    	}

    	updateSetValues[0] = 0;
    }
}

// code U_SET on CH2
void loopOperation_DacCodeU1() {
    if (uSet[1] != uSetNext[1] || updateSetValues[1]) {
    	if (reg0 & REG0_OE2_MASK) {
    		uSet[1] = uSetNext[1];
    		dacSetVoltage(1, uSet[1]);
    	} else {
    		// set voltage to zero if channel is turned off
    		dacSetVoltage(1, 0);
    	}
    }
}

// code I_SET on CH2
void loopOperation_DacCodeI1() {
    if (iSet[1] != iSetNext[1] || updateSetValues[1]) {
    	if (reg0 & REG0_OE2_MASK) {
    		iSet[1] = iSetNext[1];
    		dacSetCurrent(1, iSet[1]);
    	} else {
    		// set current to zero if channel is turned off
    		dacSetCurrent(1, 0);
    	}

    	updateSetValues[1] = 0;
    }
}

// read U_MON on CH1
void loopOperation_AdcU0() {
    uMon[0] = sdadcReadVoltage(0);
}

// read I_MON on CH1
void loopOperation_AdcI0() {
    iMon[0] = sdadcReadCurrent(0);
}

// read U_MON on CH2
void loopOperation_AdcU1() {
    uMon[1] = sdadcReadVoltage(1);
}

// read I_MON on CH2
void loopOperation_AdcI1() {
    iMon[1] = sdadcReadCurrent(1);
}

void loopOperation_Temperature() {
	adcLoop();
}

typedef void (*LoopOperation)();

LoopOperation loopOperations[] = {
  loopOperation_DacCodeU0,
  loopOperation_DacCodeI0,
  loopOperation_DacCodeU1,
  loopOperation_DacCodeI1,
  loopOperation_AdcU0,
  loopOperation_AdcI0,
  loopOperation_AdcU1,
  loopOperation_AdcI1,
  loopOperation_Temperature
};

static const int NUM_LOOP_OPERATIONS = sizeof(loopOperations) / sizeof(LoopOperation);

////////////////////////////////////////////////////////////////////////////////

void beginTransfer() {
    output[0] = reg0;

    *output_uMon0 = uMon[0];
    *output_iMon0 = iMon[0];
    *output_uMon1 = uMon[1];
    *output_iMon1 = iMon[1];

    *output_temperature0 = temperature[0];
    *output_temperature1 = temperature[1];
    *output_temperature2 = temperature[2];
    *output_CRC = HAL_CRC_Calculate(&hcrc, (uint32_t *)output, BUFFER_SIZE - 4);

    HAL_SPI_TransmitReceive_DMA(&hspi2, output, input, BUFFER_SIZE);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  transferCompleted = 1;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    transferCompleted = 1;
}

void setup() {
    readPwrGood();

    // disable outputs
    HAL_GPIO_WritePin(OE_1_GPIO_Port, OE_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OE_2_GPIO_Port, OE_2_Pin, GPIO_PIN_RESET);

    dacInit();

    sdadcInit();

    slaveSynchro();

    beginTransfer();
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
}

void loop() {
    if (transferCompleted) {
        transferCompleted = 0;

		outputEnable(0, input[0] & REG0_OE1_MASK);
		outputEnable(1, input[0] & REG0_OE2_MASK);

		uSetNext[0] = inputSetValues[0];
		iSetNext[0] = inputSetValues[1];
		uSetNext[1] = inputSetValues[2];
		iSetNext[1] = inputSetValues[3];

        beginTransfer();
    }

    LoopOperation loopOperation = loopOperations[loopOperationIndex];
    loopOperationIndex = (loopOperationIndex + 1) % NUM_LOOP_OPERATIONS;
    loopOperation();
}

