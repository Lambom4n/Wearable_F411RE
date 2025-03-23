#include "Hal.h"

#define GPIO_USED GPIOB
#define GPIO_USED_PIN_CS GPIO_PIN_15
#define GPIO_USED_PIN_RESET GPIO_PIN_14
#define GPIO_USED_PIN_INTERRUPUT GPIO_PIN_13
#define GPIO_USED_PIN_BUSY GPIO_PIN_12

extern SPI_HandleTypeDef hspi3;

RadioLibHal::RadioLibHal(const uint32_t input, const uint32_t output, const uint32_t low, const uint32_t high, const uint32_t rising, const uint32_t falling)
    : GpioModeInput(input),
      GpioModeOutput(output),
      GpioLevelLow(low),
      GpioLevelHigh(high),
      GpioInterruptRising(rising),
      GpioInterruptFalling(falling) {}

void RadioLibHal::init() {
  return;
}

void RadioLibHal::pinMode(uint32_t pin, uint32_t mode) {
  (void)pin;
  (void)mode;
}

void RadioLibHal::digitalWrite(uint32_t pin, uint32_t value) {
  //pin 4 for cs pin
  if(pin == 15) {
    HAL_GPIO_WritePin(GPIO_USED,GPIO_USED_PIN_CS, (GPIO_PinState)value);
  }
  //pin 0 for reset pin
  else if(pin == 14) {
    HAL_GPIO_WritePin(GPIO_USED,GPIO_USED_PIN_RESET, (GPIO_PinState)value);
  }
  else{
    return;
  }
}


uint32_t RadioLibHal::digitalRead(uint32_t pin) {
  //pin 1 for interrupt pin
  if(pin == 13) {
    return HAL_GPIO_ReadPin(GPIO_USED,GPIO_USED_PIN_INTERRUPUT);
  }
  //pin 5 for busy pin
  else if(pin == 12) {
    return HAL_GPIO_ReadPin(GPIO_USED,GPIO_USED_PIN_BUSY);
  }
  else {
    return 1;
  }
}


void RadioLibHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
  (void)interruptNum;
  (void)interruptCb;
  (void)mode;
}

void RadioLibHal::detachInterrupt(uint32_t interruptNum) {
  (void)interruptNum;
}

void RadioLibHal::delay(RadioLibTime_t ms) {
  HAL_Delay(ms);
}

void RadioLibHal::delayMicroseconds(RadioLibTime_t us) {
  uint32_t ms = (uint32_t)((uint32_t)us / 1000);
  HAL_Delay(ms);
}

RadioLibTime_t RadioLibHal::millis() {
  return HAL_GetTick();
}

RadioLibTime_t RadioLibHal::micros() {
  return HAL_GetTick()*1000;
}

long RadioLibHal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
  (void)pin;
  (void)state;
  (void)timeout;
  return(0);
}

void RadioLibHal::spiBeginTransaction() {
  return;
}

void RadioLibHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
  HAL_SPI_TransmitReceive(&hspi3, out, in, len, 1000);
}

void RadioLibHal::spiEndTransaction() {
  return;
}

void RadioLibHal::spiEnd() {
  return;
}



void RadioLibHal::term() {
  HAL_SPI_DeInit(&hspi3);
}

void RadioLibHal::tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration) {
  (void)pin;
  (void)frequency;
  (void)duration;
}

void RadioLibHal::noTone(uint32_t pin) {
  (void)pin;
}

void RadioLibHal::yield() {
  return;
}

uint32_t RadioLibHal::pinToInterrupt(uint32_t pin) {
  (void)pin;
  return EXTI15_10_IRQn;
}
