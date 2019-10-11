#include "main.h"

////////////////////////////////////////////////////////////////////////////////

extern SPI_HandleTypeDef hspi1;
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
int loadDac;

////////////////////////////////////////////////////////////////////////////////

int isInterrupt() {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t reg0;

// just mark for output enable/disable and wait for DIB_SYNC to perform actual operation
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == DIB_SYNC_Pin) {
        // DIB_SYNC received, enable/disable output

    	if (transferCompleted) {
            uint8_t cmd = input[0];
            if (cmd & 0x80) {
				outputEnable(0, cmd & REG0_OE1_MASK);
				outputEnable(1, cmd & REG0_OE2_MASK);
            }
        }

        HAL_GPIO_WritePin(OE_1_GPIO_Port, OE_1_Pin, reg0 & REG0_OE1_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OE_2_GPIO_Port, OE_2_Pin, reg0 & REG0_OE2_MASK ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    }
}

////////////////////////////////////////////////////////////////////////////////

void DAC_SPI_Transmit(uint8_t r0, uint8_t r1, uint8_t r2) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // DAC CS active low
    uint8_t input[3] = { r0, r1, r2 };
    if (HAL_SPI_Transmit(&hspi1, input, 3, 10) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // DAC CS active high
}

void dacInit() {
    // https://datasheets.maximintegrated.com/en/ds/MAX5713-MAX5715.pdf

    // DAC configuration SW RESET
    DAC_SPI_Transmit(0b01010001, 0, 0);
    HAL_Delay(1);

    // DAC configuration POWER
    DAC_SPI_Transmit(0b01000000, 0b00001111, 0);
    HAL_Delay(1);

    // DAC configuration SW CLEAR
    DAC_SPI_Transmit(0b01010000, 0, 0);
    HAL_Delay(1);

    // DAC set internal ref. voltage to 2.5V
    DAC_SPI_Transmit(0b01110101, 0, 0);
    HAL_Delay(1);
}

void dac_CODEn(uint8_t dacSelection, uint16_t value) {
    DAC_SPI_Transmit(0b00000000 | dacSelection, value >> 4, (value & 0x0F) << 4);
}

void dac_LOAD_ALL() {
    DAC_SPI_Transmit(0b10000001, 0, 0);
}

void dac_CODEn_LODEn(uint8_t dacSelection, uint16_t value) {
    DAC_SPI_Transmit(0b00110000 | dacSelection, value >> 4, (value & 0x0F) << 4);
}

uint8_t getVoltageDacSelection(uint8_t iChannel) {
    return iChannel == 0 ? 0 /* DAC A */ : 3 /* DAC D */;
}

uint8_t getCurrentDacSelection(uint8_t iChannel) {
    return iChannel == 0 ? 1 /* DAC A */ : 2 /* DAC D */;
}

void dacSetVoltage(uint8_t iChannel, uint16_t value) {
    dac_CODEn_LODEn(getVoltageDacSelection(iChannel), 4095 - value);
}

void dacSetCurrent(uint8_t iChannel, uint16_t value) {
    dac_CODEn_LODEn(getCurrentDacSelection(iChannel), value);
}

void dacCodeVoltage(uint8_t iChannel, uint16_t value) {
    dac_CODEn(getVoltageDacSelection(iChannel), 4095 - value);
}

void dacCodeCurrent(uint8_t iChannel, uint16_t value) {
    dac_CODEn(getCurrentDacSelection(iChannel), value);
}

void dacSetAllZero() {
    dacCodeVoltage(0, 0);
    dacCodeVoltage(1, 0);
    dacCodeCurrent(0, 0);
    dacCodeCurrent(1, 0);
    dac_LOAD_ALL();
}

void dacSetAllHalf() {
    // CODE_ALL_LOAD_ALL
    DAC_SPI_Transmit(0b10000010, 0b10100000, 0);
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

    return value;
}

uint16_t sdadcReadVoltage(int iChannel) {
    return sdadcRead(iChannel == 0 ? SDADC_CHANNEL_6 : SDADC_CHANNEL_4);
}

uint16_t sdadcReadCurrent(int iChannel) {
    return sdadcRead(iChannel == 0 ? SDADC_CHANNEL_5 : SDADC_CHANNEL_8);
}

////////////////////////////////////////////////////////////////////////////////

uint16_t adcConvertedValues[3];
int adcTransferCompleted = 1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcTransferCompleted = 1;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
	adcTransferCompleted = 2;
}

void adcInit() {
}

void adcLoop() {
    if (adcTransferCompleted > 0) {
    	if (adcTransferCompleted == 1) {
    		temperature[0] = adcConvertedValues[0];
    		temperature[1] = adcConvertedValues[1];
    		temperature[2] = adcConvertedValues[2];
    	}

    	adcTransferCompleted = 0;

    	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcConvertedValues, 3);
    }
}

////////////////////////////////////////////////////////////////////////////////

#define SPI_SLAVE_SYNBYTE         0x53
#define SPI_MASTER_SYNBYTE        0xAC

void slaveSynchro(void) {
    uint8_t txackbyte = SPI_SLAVE_SYNBYTE, rxackbyte = 0x00;
    do {
        if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&txackbyte, (uint8_t *)&rxackbyte, 1, HAL_MAX_DELAY) != HAL_OK) {
            Error_Handler();
        }
    } while (rxackbyte != SPI_MASTER_SYNBYTE);
}

////////////////////////////////////////////////////////////////////////////////

// code U_SET on CH1
void loopOperation_DacCodeU0() {
    if (uSet[0] != uSetNext[0] || updateSetValues[0]) {
    	if (reg0 & REG0_OE1_MASK) {
    		uSet[0] = uSetNext[0];
    		dacCodeVoltage(0, uSet[0]);
    	} else {
    		// set voltage to zero if channel is turned off
    		dacCodeVoltage(0, 0);
    	}
        loadDac = 1;
    }
}

// code I_SET on CH1
void loopOperation_DacCodeI0() {
    if (iSet[0] != iSetNext[0] || updateSetValues[0]) {
    	if (reg0 & REG0_OE1_MASK) {
    		iSet[0] = iSetNext[0];
    		dacCodeCurrent(0, iSet[0]);
    	} else {
    		// set current to zero if channel is turned off
    		dacCodeCurrent(0, 0);
    	}
        loadDac = 1;
    }
}

// code U_SET on CH2
void loopOperation_DacCodeU1() {
    if (uSet[1] != uSetNext[1] || updateSetValues[1]) {
    	if (reg0 & REG0_OE2_MASK) {
    		uSet[1] = uSetNext[1];
    		dacCodeVoltage(1, uSet[1]);
    	} else {
    		// set voltage to zero if channel is turned off
    		dacCodeVoltage(1, 0);
    	}
        loadDac = 1;
    }
}

// code I_SET on CH2
void loopOperation_DacCodeI1() {
    if (iSet[1] != iSetNext[1] || updateSetValues[1]) {
    	if (reg0 & REG0_OE2_MASK) {
    		iSet[1] = iSetNext[1];
    		dacCodeCurrent(1, iSet[1]);
    	} else {
    		// set current to zero if channel is turned off
    		dacCodeCurrent(1, 0);
    	}
        loadDac = 1;
    }
}

// load all coded SET values
void loopOperation_DacLoadAll() {
    if (loadDac) {
        dac_LOAD_ALL();
        loadDac = 0;
    	updateSetValues[0] = 0;
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
  loopOperation_DacLoadAll,
  loopOperation_AdcU0,
  loopOperation_AdcI0,
  loopOperation_AdcU1,
  loopOperation_AdcI1,
  loopOperation_Temperature
};

static const int NUM_LOOP_OPERATIONS = sizeof(loopOperations) / sizeof(LoopOperation);

////////////////////////////////////////////////////////////////////////////////

void readPwrGood() {
    if (HAL_GPIO_ReadPin(PWRGOOD_GPIO_Port, PWRGOOD_Pin) == GPIO_PIN_SET) {
        reg0 |= REG0_PWRGOOD_MASK;
    } else {
        reg0 &= ~REG0_PWRGOOD_MASK;
    }
}

void beginTransfer() {
    readPwrGood();

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
    // disable outputs
    HAL_GPIO_WritePin(OE_1_GPIO_Port, OE_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OE_2_GPIO_Port, OE_2_Pin, GPIO_PIN_RESET);

    dacInit();
    dacSetAllZero();

    sdadcInit();

    adcInit();

    slaveSynchro();

    beginTransfer();
    HAL_GPIO_WritePin(DIB_IRQ_GPIO_Port, DIB_IRQ_Pin, GPIO_PIN_SET);
}

void loop() {
    if (transferCompleted) {
        transferCompleted = 0;

        uint8_t cmd = input[0];

        if (cmd & 0x80) {
            outputEnable(0, cmd & REG0_OE1_MASK);
            outputEnable(1, cmd & REG0_OE2_MASK);

            uSetNext[0] = inputSetValues[0];
            iSetNext[0] = inputSetValues[1];
            uSetNext[1] = inputSetValues[2];
            iSetNext[1] = inputSetValues[3];
        } else if (cmd & 0x40) {
            cmd &= ~0xC0;

            uint16_t param = (input[2] << 8) | input[1];

            switch (cmd) {
            case 10: outputEnable (0, 0); break;
            case 11: outputEnable (0, 1); break;
            case 12: ccLED        (0, 0); break;
            case 13: ccLED        (0, 1); break;
            case 14: uSetNext[0] = param; break;
            case 15: iSetNext[0] = param; break;

            case 20: outputEnable (1, 0); break;
            case 21: outputEnable (1, 1); break;
            case 22: ccLED        (1, 0); break;
            case 23: ccLED        (1, 1); break;
            case 24: uSetNext[1] = param; break;
            case 25: iSetNext[1] = param; break;
            }
        }

        beginTransfer();
    }

    LoopOperation loopOperation = loopOperations[loopOperationIndex];
    loopOperationIndex = (loopOperationIndex + 1) % NUM_LOOP_OPERATIONS;
    loopOperation();
}

