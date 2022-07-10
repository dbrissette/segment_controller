# segment_controller

Code installed on the Pico RP2040, that has an I2C periferial connection to the ESP32.  
It receives two bytes, an offset and a value that updates a digit that is displayed by a seven segement display.

ESP32 ==> I2C ==> Pico ( Set value of display )
                  Pico ==> SPI* ======> Shift Registers (Rotate value of display and the pin offset that will enable the MOSFET on the 7 Segement Anode Pin) 
                       ==> GPIO_RCK ==> Shift Registers (Dumps all the shift register values, to pull the enabled pins low,
                                                         one display will be illuminated with the value that was programmed into the shift registers
                                                         in the previous step)
                                                         
* The clock and data out pins are the only features used.  A separate GPIO pin is used to trigger the Register Clock (RCK) that is used to push the bits out. 
