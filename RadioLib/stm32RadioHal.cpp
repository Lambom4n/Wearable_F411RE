#include "stm32RadioHal.h"

#define GPIO_USED GPIOB
#define GPIO_USED_PIN_CS GPIO_PIN_15
#define GPIO_USED_PIN_RESET GPIO_PIN_14
#define GPIO_USED_PIN_INTERRUPUT GPIO_PIN_13
#define GPIO_USED_PIN_BUSY GPIO_PIN_12

extern SPI_HandleTypeDef hspi3;

void STM32RadioHal::init()
{
    return;
}

void STM32RadioHal::pinMode(uint32_t pin, uint32_t mode)
{
    (void)pin;
    (void)mode;
}

void STM32RadioHal::digitalWrite(uint32_t pin, uint32_t value)
{
    // pin 15 for cs pin
    if (pin == 15)
    {
        HAL_GPIO_WritePin(GPIO_USED, GPIO_USED_PIN_CS, (GPIO_PinState)value);
    }
    // pin 14 for reset pin
    else if (pin == 14)
    {
        HAL_GPIO_WritePin(GPIO_USED, GPIO_USED_PIN_RESET, (GPIO_PinState)value);
    }
    else
    {
        return;
    }
}

uint32_t STM32RadioHal::digitalRead(uint32_t pin)
{
    // pin 1 for interrupt pin
    if (pin == 13)
    {
        return HAL_GPIO_ReadPin(GPIO_USED, GPIO_USED_PIN_INTERRUPUT);
    }
    // pin 5 for busy pin
    else if (pin == 12)
    {
        return HAL_GPIO_ReadPin(GPIO_USED, GPIO_USED_PIN_BUSY);
    }
    else
    {
        return 1;
    }
}

void STM32RadioHal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode)
{
    (void)interruptNum;
    (void)interruptCb;
    (void)mode;
}

void STM32RadioHal::detachInterrupt(uint32_t interruptNum)
{
    (void)interruptNum;
}

void STM32RadioHal::delay(RadioLibTime_t ms)
{
    HAL_Delay(ms);
}

void STM32RadioHal::delayMicroseconds(RadioLibTime_t us)
{
    uint32_t ms = (uint32_t)((uint32_t)us / 1000);
    HAL_Delay(ms);
}

RadioLibTime_t STM32RadioHal::millis()
{
    return HAL_GetTick();
}

RadioLibTime_t STM32RadioHal::micros()
{
    return HAL_GetTick() * 1000;
}

long STM32RadioHal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout)
{
    (void)pin;
    (void)state;
    (void)timeout;
    return (0);
}

void STM32RadioHal::spiBegin()
{
    return;
}

void STM32RadioHal::spiBeginTransaction()
{
    return;
}

void STM32RadioHal::spiTransfer(uint8_t *out, size_t len, uint8_t *in)
{
    HAL_SPI_TransmitReceive(&hspi3, out, in, len, 1000);
}

void STM32RadioHal::spiEndTransaction()
{
    return;
}

void STM32RadioHal::spiEnd()
{
    return;
}

void STM32RadioHal::term()
{
    HAL_SPI_DeInit(&hspi3);
}

void STM32RadioHal::tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration)
{
    (void)pin;
    (void)frequency;
    (void)duration;
}

void STM32RadioHal::noTone(uint32_t pin)
{
    (void)pin;
}

void STM32RadioHal::yield()
{
    return;
}

uint32_t STM32RadioHal::pinToInterrupt(uint32_t pin)
{
    (void)pin;
    return EXTI15_10_IRQn;
}