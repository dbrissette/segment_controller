/**
 * 
 * 
 */


// Usage:
//
// When writing data to the pico the first data byte is the offset in the segment_values array.
// The second data byte is the value in the offset location in segment_values array.
// segment_values[1st byte] = 2nd byte;
//

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/util/queue.h" 

// define I2C addresses to be used for this peripheral
#define I2C0_PERIPHERAL_ADDR 0x42  
// GPIO pins to use for SPI
#define PIN_SCK     2  // Pin 4
#define PIN_MOSI    3  // Pin 5
#define PIN_MISO    4  // Pin 6
#define PIN_CS      5  // Pin 7
#define GPIO_RCK    6  // Pin 9
// GPIO pins to use for I2C
#define GPIO_SDA0  12  // Pin 16
#define GPIO_SCK0  13  // Pin 17
#define LOW         0
#define HIGH        1
// CLK_FAST: actually set to clk_peri (= 125.0 MHz) / N,
// which is determined by spi_set_baudrate() in pico-sdk/src/rp2_common/hardware_spi/spi.c
#define CLK_FAST	(50 * 100 * 100)
// Display Setup:
/* DISPLAY_COUNT = 8 x Number of NUM_SHIFT_REGISTERS */
static const int8_t DISPLAY_COUNT = 48;
static const int8_t LAST_DISPLAY_COUNT_INDEX = DISPLAY_COUNT - 1;
static const int8_t NUM_SHIFT_REGISTERS = 6; // Total number of Shift Registers
static const int8_t LAST_SHIFT_REGISTER_INDEX = NUM_SHIFT_REGISTERS - 1;

char offset = 0;
uint8_t loop_counter = 0;
// I2C queue, used to leave ISR context
static queue_t i2c_queue;

// using struct as an example, but primitive types can be used too
typedef struct payload {
    char offset;
    char value;
} payload_t;

void core1_entry();
uint8_t DIGIT_TO_7SEG_ENCODER(uint8_t digit);

// Interrupt handler 
void i2c0_irq_handler() {

    // Get interrupt status
    uint32_t status = i2c0->hw->intr_stat;


    // Check to see if we have received data from the I2C controller
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        // Read the data (this will clear the interrupt)
        uint32_t data = i2c0->hw->data_cmd;
        // Check if this is the 1st byte we have received
        if (data & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            offset = 0;
            loop_counter = 1;

        } else {
            if (loop_counter == 1) {
                offset = data;
            }
            if (loop_counter == 2) {
                payload_t payload = {
                    .offset = offset,
                    .value = data
                };
                if (!queue_try_add(&i2c_queue, &payload)) {
                    printf("FIFO was full\n");
                }
            }
            loop_counter++;
        }
    }
}

// Main loop - initilises system and then loops while interrupts get on with processing the data
int main() {
    stdio_init_all();

    // Setup I2C0 as slave (peripheral)
    i2c_init(i2c0, 100 * 1000);
    i2c_set_slave_mode(i2c0, true, I2C0_PERIPHERAL_ADDR);

    // Setup GPIO pins to use and add pull up resistors
    gpio_set_function(GPIO_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCK0, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA0);
    gpio_pull_up(GPIO_SCK0);

    // Enable the I2C interrupts we want to process
    i2c0->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);

    // Set up the interrupt handler to service I2C interrupts
    irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);

    // Enable I2C interrupt
    irq_set_enabled(I2C0_IRQ, true);


    queue_init(&i2c_queue, sizeof(payload_t), 8);

    multicore_launch_core1(core1_entry);

    // Do nothing in main loop
    while (true) {
        tight_loop_contents();
    }
    return 0;
}

void core1_entry() {
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // Enable SPI 0 at X MHz and connect to GPIOs
    spi_init(spi0, CLK_FAST);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_init(GPIO_RCK);
    gpio_pull_down(GPIO_RCK);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(GPIO_RCK, GPIO_OUT);
    gpio_put(PIN_CS, HIGH);
    gpio_put(GPIO_RCK, LOW);
    printf("SPI initialised\n");

    uint8_t sr_data = 0b00000000;
    uint8_t active_register = 0;
    // char segment_values[DISPLAY_COUNT] = { [0 ... LAST_DISPLAY_COUNT_INDEX] = 0b00000000 };
    char segment_values[48] = { [0 ... 47] = 0b00000000 };
    // buf is the bytes that are pushed out to the wire via the SPI DO interface.
    // buf size is based on the total number of shift registers.
    // char buf[NUM_SHIFT_REGISTERS] = { [0 ... LAST_SHIFT_REGISTER_INDEX] = 0b00000000 };
    char buf[6] = { [0 ... 5] = 0b00000000 };
    while(1) {
        gpio_put(GPIO_RCK, LOW);
        payload_t payload;
        for(int8_t index = 8; index < DISPLAY_COUNT; index++) 
        {
            if (queue_try_remove(&i2c_queue, &payload)) 
            {
                // printf("Got offset: %i, value: %i\n", payload.offset, payload.value);
                segment_values[payload.offset] = DIGIT_TO_7SEG_ENCODER(payload.value);
                uint32_t baud_rate = spi_get_baudrate(spi0);
            }
            buf[0] = segment_values[index];
            active_register = index / 8;
            uint8_t register_pin = index % 8;
            for (int8_t sr_index = 5; sr_index > 0; sr_index--)
            {
                sr_data = 0b00000000;
                if(active_register == sr_index) {
                    sr_data = (1 << register_pin);
                }
                buf[sr_index] = sr_data;
            }
            spi_write_blocking(spi0, buf, NUM_SHIFT_REGISTERS);
            gpio_put(GPIO_RCK, HIGH);
            sleep_us(5);
            gpio_put(GPIO_RCK, LOW);
            sleep_us(100);
            if(index == DISPLAY_COUNT-1) {
                index = -1;
            }
        }
    }
}

/* ----------------------------------------------------------------------------
 * Function      : unsigned char DIGIT_TO_7SEG_ENCODER(unsigned char digit)
 * ----------------------------------------------------------------------------
 * Description   : Receive digit and return 7-segment encoded binary value.
 * Inputs        : unsigned positive number between 0 and 9
 * Outputs       : binary number representing the digit on a 7-Segment display, Common Anode
 * Assumptions   : Common Anode 7-Segment display
 * ------------------------------------------------------------------------- */
uint8_t DIGIT_TO_7SEG_ENCODER(uint8_t digit)
// unsigned char DIGIT_TO_7SEG_ENCODER(unsigned char digit)
{
	uint8_t segval = 0b00000000;
	// unsigned char segval = 0b00000000;
    // ESP_LOGE(tag, "DIGIT_TO_7SEG_ENCODER: %d", digit);

	switch(digit)
	{
		case 0: segval = 0b00111111;
				break;
		case 1:	segval = 0b00000110;
				break;
		case 2:	segval = 0b01011011;
				break;
		case 3:	segval = 0b01001111;
				break;
		case 4:	segval = 0b01100110;
				break;
		case 5:	segval = 0b01101101;
				break;
		case 6:	segval = 0b01111101;
				break;
		case 7:	segval = 0b00000111;
				break;
		case 8:	segval = 0b01111111;
				break;
		case 9:	segval = 0b01101111;
                break;
		case 10: segval = 0b00000000;
				break;
	}
    // ESP_LOGE(tag, "DIGIT_TO_7SEG_ENCODER segval: %d", segval);
	return segval;
}
