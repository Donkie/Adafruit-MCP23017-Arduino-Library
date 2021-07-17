/*!
 * @file Adafruit_MCP23X18.cpp
 *
 * @mainpage Adafruit MCP23X08/17 Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the MCP23008/17 I2C and MCP23S08/17 SPI port
 * expanders.
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Carter Nelson for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_MCP23X18.h"

/**************************************************************************/
/*!
  @brief Configures the specified pin to behave either as an input or an output.
  @param pin the Arduino pin number to set the mode of
  @param mode INPUT, OUTPUT, INPUT_PULLUP or OUTPUT_OPEN_DRAIN
*/
/**************************************************************************/
void Adafruit_MCP23X18::pinMode(uint8_t pin, uint8_t mode) {
  Adafruit_BusIO_Register IODIR(i2c_dev, spi_dev, MCP23XXX_SPIREG,
                                getRegister(MCP23XXX_IODIR, MCP_PORT(pin)));
  Adafruit_BusIO_Register GPPU(i2c_dev, spi_dev, MCP23XXX_SPIREG,
                               getRegister(MCP23XXX_GPPU, MCP_PORT(pin)));
  Adafruit_BusIO_RegisterBits dir_bit(&IODIR, 1, pin % 8);
  Adafruit_BusIO_RegisterBits pullup_bit(&GPPU, 1, pin % 8);

  dir_bit.write((mode == OUTPUT || mode == OUTPUT_OPEN_DRAIN) ? 0 : 1);
  pullup_bit.write((mode == INPUT_PULLUP || mode == OUTPUT) ? 1 : 0);
}

/**************************************************************************/
/*!
  @brief Configure the interrupt system.
  @param mirroring true to OR both INTA and INTB pins.
  @param open true for open drain output, false for active drive output.
  @param polarity HIGH or LOW
*/
/**************************************************************************/
void Adafruit_MCP23X18::setupInterrupts(bool mirroring, bool openDrain,
                                        uint8_t polarity) {
  Adafruit_BusIO_Register GPINTEN(i2c_dev, spi_dev, MCP23XXX_SPIREG,
                                  getRegister(MCP23XXX_IOCON));
  Adafruit_BusIO_RegisterBits mirror_bit(&GPINTEN, 1, 6);
  Adafruit_BusIO_RegisterBits openDrain_bit(&GPINTEN, 1, 2);
  Adafruit_BusIO_RegisterBits polarity_bit(&GPINTEN, 1, 1);
  Adafruit_BusIO_RegisterBits inputClearControl_bit(&GPINTEN, 1, 0);

  mirror_bit.write(mirroring ? 1 : 0);
  openDrain_bit.write(openDrain ? 1 : 0);
  polarity_bit.write((polarity == HIGH) ? 1 : 0);
  // Setting input clear control to 1 makes it behave the same way as the
  // MCP23X17 does
  inputClearControl_bit.write(1);
}
