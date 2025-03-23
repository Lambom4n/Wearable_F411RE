#if !defined(STM32_RADIO_HAL_H)
#define STM32_RADIO_HAL_H

#include "Hal.h"

class STM32RadioHal : public RadioLibHal {
  public:
    STM32RadioHal(const uint32_t input, const uint32_t output, const uint32_t low, const uint32_t high, const uint32_t rising, const uint32_t falling)
        : RadioLibHal(input, output, low, high, rising, falling) {}
    /*!
      \brief GPIO pin mode (input/output/...) configuration method.
      Must be implemented by the platform-specific hardware abstraction!
      \param pin Pin to be changed (platform-specific).
      \param mode Mode to be set (platform-specific).
    */
    void pinMode(uint32_t pin, uint32_t mode);

   /*!
     \brief Digital write method.
     Must be implemented by the platform-specific hardware abstraction!
     \param pin Pin to be changed (platform-specific).
     \param value Value to set (platform-specific).
   */
    void digitalWrite(uint32_t pin, uint32_t value);

   /*!
     \brief Digital read method.
     Must be implemented by the platform-specific hardware abstraction!
     \param pin Pin to be changed (platform-specific).
     \returns Value read on the pin (platform-specific).
   */
    uint32_t digitalRead(uint32_t pin);
   
   /*!
     \brief Method to attach function to an external interrupt.
     Must be implemented by the platform-specific hardware abstraction!
     \param interruptNum Interrupt number to attach to (platform-specific).
     \param interruptCb Interrupt service routine to execute.
     \param mode Rising/falling mode (platform-specific).
   */
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode);

   /*!
     \brief Method to detach function from an external interrupt.
     Must be implemented by the platform-specific hardware abstraction!
     \param interruptNum Interrupt number to detach from (platform-specific).
   */
    void detachInterrupt(uint32_t interruptNum);

   /*!
     \brief Blocking wait function.
     Must be implemented by the platform-specific hardware abstraction!
     \param ms Number of milliseconds to wait.
   */
    void delay(RadioLibTime_t ms);
   
   /*!
     \brief Blocking microsecond wait function.
     Must be implemented by the platform-specific hardware abstraction!
     \param us Number of microseconds to wait
     */
     void delayMicroseconds(RadioLibTime_t us);
   
   /*!
     \brief Get number of milliseconds since start.
     Must be implemented by the platform-specific hardware abstraction!
     \returns Number of milliseconds since start
     */
     RadioLibTime_t millis();
   
   /*!
     \brief Get number of microseconds since start.
     Must be implemented by the platform-specific hardware abstraction!
     \returns Number of microseconds since start
     */
     RadioLibTime_t micros();
   
   /*!
     \brief Measure the length of incoming digital pulse in microseconds.
     Must be implemented by the platform-specific hardware abstraction!
     \param pin Pin to measure on (platform-specific).
     \param state Pin level to monitor (platform-specific).
     \param timeout Timeout in microseconds.
     \returns Pulse length in microseconds, or 0 if the pulse did not start before timeout.
   */
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout);

   /*!
     \brief SPI initialization method.
   */
    void spiBegin();

   /*!
     \brief Method to start SPI transaction
     */
     void spiBeginTransaction();

   /*!
     \brief Method to transfer buffer over SPI.
     \param out Buffer to send.
     \param len Number of data to send or receive.
     \param in Buffer to save received data into.
   */
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in);

   /*!
     \brief Method to end SPI transaction.
   */
   virtual void spiEndTransaction();

   /*!
     \brief SPI termination method.
   */
    void spiEnd();

   // virtual methods - these may or may not exists on a given platform
   // they exist in this implementation, but do nothing

   /*!
     \brief Module initialization method.
     This will be called by all radio modules at the beginning of startup.
     Can be used to e.g., initialize SPI interface.
   */
    void init();

   /*!
     \brief Module termination method.
     This will be called by all radio modules when the destructor is called.
     Can be used to e.g., stop SPI interface
     */
     void term();

   /*!
     \brief Method to produce a square-wave with 50% duty cycle ("tone") of a given frequency at some pin.
     \param pin Pin to be used as the output.
     \param frequency Frequency of the square wave.
     \param duration Duration of the tone in ms. When set to 0, the tone will be infinite
     */
     void tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration = 0);

   /*!
     \brief Method to stop producing a tone.
     \param pin Pin which is currently producing the tone.
   */
   virtual void noTone(uint32_t pin);
   
   /*!
     \brief Yield method, called from long loops in multi-threaded environment (to prevent blocking other threads)
    */
     void yield();
   
   /*!
     \brief Function to convert from pin number to interrupt number.
     \param pin Pin to convert from.
     \returns The interrupt number of a given pin
     */
     uint32_t pinToInterrupt(uint32_t pin);

};

#endif