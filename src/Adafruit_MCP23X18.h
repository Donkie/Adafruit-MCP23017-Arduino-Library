/*!
 * @file Adafruit_MCP23X18.h
 */

#ifndef __Adafruit_MCP23X18_H__
#define __Adafruit_MCP23X18_H__

#include "Adafruit_MCP23XXX.h"

#ifndef OUTPUT_OPEN_DRAIN
#define OUTPUT_OPEN_DRAIN 0x03
#endif

/**************************************************************************/
/*!
    @brief  Class for MCP23018 I2C and MCP23S18 SPI variants.
*/
/**************************************************************************/
class Adafruit_MCP23X18 : public Adafruit_MCP23X17 {
public:
  void pinMode(uint8_t pin, uint8_t mode) override;

  void setupInterrupts(bool mirroring, bool openDrain,
                       uint8_t polarity) override;
};

#endif