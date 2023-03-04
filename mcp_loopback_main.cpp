/**********************************************************************
 * Example code for MCP_CAN library, ESP32 port
 * CAN bus loopback test for ESP32 with 2x MCP2515 CAN drivers.
 * Refer to MCP_CAN by Corey J Fowler for more examples.
 * 
 * 2023-03-03 Dan Goldwater
***********************************************************************/

#include <stdio.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "bootloader_random.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "mcp_can.hpp"

static const char *TAG = "loopback";

// 2x LEDs
#define BLINK0_GPIO (gpio_num_t)25
#define BLINK1_GPIO (gpio_num_t)26

// HUZZAH32 SPI bus
#define SPI_MISO_PIN (gpio_num_t)19
#define SPI_MOSI_PIN (gpio_num_t)18  
#define SPI_SCK_PIN (gpio_num_t)5

// MCP2515 clock is driven from the ESP32
#define CAN_CLK_pin (gpio_num_t)12

// Hookups to 2x MCP2515 chips:
#define CAN0_RESET_pin (gpio_num_t)13
#define CAN0_INT_pin (gpio_num_t)14
#define CAN0_CS_pin (gpio_num_t)15
#define CAN1_RESET_pin (gpio_num_t)27
#define CAN1_INT_pin (gpio_num_t)32
#define CAN1_CS_pin (gpio_num_t)33

static uint8_t s_led_state = 0;

// Our hardware has 2x MCP2515 chips on 1 SPI bus.
static spi_device_handle_t SPI0;
static spi_device_handle_t SPI1;

// MCP2515 receive buffers
unsigned long rxId;
INT8U rxLen;
INT8U rxBuf[8];
char rxString[128];
// transmit buffers
INT8U txBuf[8];

// CAN objects
MCP_CAN CAN0(&SPI0, CAN0_CS_pin);
MCP_CAN CAN1(&SPI1, CAN1_CS_pin);

static void setup(void)
{
    bootloader_random_enable();  // seed random numbers

    // Set up GPIO pins on HUZZAH32
    ESP_LOGI(TAG, "Configuring GPIO");
    gpio_reset_pin(BLINK0_GPIO);
    gpio_reset_pin(BLINK1_GPIO);
    gpio_set_direction(BLINK0_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLINK1_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(CAN0_INT_pin);
    gpio_reset_pin(CAN1_INT_pin);
    gpio_set_direction(CAN0_INT_pin, GPIO_MODE_INPUT);
    gpio_set_direction(CAN1_INT_pin, GPIO_MODE_INPUT);

    gpio_reset_pin(CAN0_CS_pin);
    gpio_reset_pin(CAN1_CS_pin);
    gpio_set_level(CAN0_CS_pin, 1);
    gpio_set_level(CAN1_CS_pin, 1);
    gpio_set_direction(CAN0_CS_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(CAN1_CS_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN0_CS_pin, 1);
    gpio_set_level(CAN1_CS_pin, 1);

    gpio_reset_pin(CAN_CLK_pin);
    gpio_set_direction(CAN_CLK_pin, GPIO_MODE_OUTPUT);

    // reset the MCP2515 chips
    gpio_reset_pin(CAN0_RESET_pin);
    gpio_reset_pin(CAN1_RESET_pin);
    gpio_set_direction(CAN0_RESET_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(CAN1_RESET_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN0_RESET_pin, 0);
    gpio_set_level(CAN1_RESET_pin, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(CAN0_RESET_pin, 1);
    gpio_set_level(CAN1_RESET_pin, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);


    // set up 8MHz clock for MCP2515 using ESP32's hardware PWM function
    ESP_LOGI(TAG, "Configuring SPI clock");
    ledc_timer_config_t ledc_timer = {};
        ledc_timer.speed_mode       = LEDC_HIGH_SPEED_MODE;
        ledc_timer.timer_num        = LEDC_TIMER_0;
        ledc_timer.duty_resolution  = LEDC_TIMER_2_BIT;
        ledc_timer.freq_hz          = 8000000;
        ledc_timer.clk_cfg          = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {};
        ledc_channel.speed_mode     = LEDC_HIGH_SPEED_MODE;
        ledc_channel.channel        = LEDC_CHANNEL_0;
        ledc_channel.timer_sel      = LEDC_TIMER_0;
        ledc_channel.intr_type      = LEDC_INTR_DISABLE;
        ledc_channel.gpio_num       = CAN_CLK_pin;
        ledc_channel.duty           = 2;   // Set duty to ~50%
        ledc_channel.hpoint         = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    uint32_t f = ledc_get_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
    ESP_LOGI(TAG, "MSP2515 clock frequency set: %lu", f);


    // SPI bus initialization
    ESP_LOGI(TAG, "Configuring SPI");

    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = SPI_MISO_PIN;
    buscfg.mosi_io_num = SPI_MOSI_PIN;
    buscfg.sclk_io_num = SPI_SCK_PIN;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    esp_err_t ret;
    // CAN bus messages are 8 bytes maximum, so we set SPI_DMA_DISABLED
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);  // should return ESP_OK

    // SPI slave device configuration
    spi_device_interface_config_t devcfg0 = {};
    devcfg0.clock_speed_hz = 4000000;
    devcfg0.mode = 0;
    devcfg0.spics_io_num = CAN0_CS_pin;
    devcfg0.queue_size = 7;

    spi_device_interface_config_t devcfg1 = {};
    devcfg1.clock_speed_hz = 4000000;
    devcfg1.mode = 0;
    devcfg1.spics_io_num = CAN1_CS_pin;
    devcfg1.queue_size = 7;

    // attach the 2x MCP2515 slaves to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg0, &SPI0);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(HSPI_HOST, &devcfg1, &SPI1);
    ESP_ERROR_CHECK(ret);


    ESP_LOGI(TAG, "Configuring CAN 0");
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        ESP_LOGI(TAG, "CAN0: Init OK");
    } else {
        ESP_LOGI(TAG, "CAN0: Init Fail");
    }
    if(CAN0.setMode(MCP_NORMAL) == MCP_NORMAL) {
        ESP_LOGI(TAG, "CAN0: Set mode OK");
    } else {
        ESP_LOGI(TAG, "CAN0: Set mode fail");
    }

    ESP_LOGI(TAG, "Configuring CAN 1");
    if(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        ESP_LOGI(TAG, "CAN1: Init OK");
    } else {
        ESP_LOGI(TAG, "CAN1: Init Fail");
    }
    if(CAN1.setMode(MCP_NORMAL) == MCP_NORMAL) {
        ESP_LOGI(TAG, "CAN1: Set mode OK");
    } else {
        ESP_LOGI(TAG, "CAN1: Set mode fail");
    }
}

void the_real_main(void) {
    uint64_t t1, t2;

    ESP_LOGI(TAG, "Starting Loopback");
    
    // check receiver every 10ms.  send a new message and toggle LED every 1000ms.

    t1 = esp_timer_get_time();
    while (1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);  // milliseconds
        t2 = esp_timer_get_time();
        
        if((t2 - t1) > 1000000) {  // microseconds
            t1 = esp_timer_get_time();

            ESP_LOGI(TAG, "Turning the LED %s", s_led_state == true ? "ON" : "OFF");
            /* Toggle the LED state */
            s_led_state = !s_led_state;
            gpio_set_level(BLINK0_GPIO, s_led_state);
            gpio_set_level(BLINK1_GPIO, !s_led_state);
    

            // SEND ON CAN1 PERIODICALLY //
            sprintf(rxString, "Random data: ");
            for(uint8_t i=0; i<8; i++) {
                txBuf[i] = esp_random();  // returns a uint32 of random data
                sprintf(rxString + strlen(rxString), "%x ", txBuf[i]);
            }
            ESP_LOGI(TAG, "%s", rxString);

            // message ID, frame type (0 = standard, 1 = extended), data length, data array
            uint8_t sendStatus = CAN1.sendMsgBuf(0x10101, 1, 8, txBuf);
            if(sendStatus == CAN_OK) {
                ESP_LOGI(TAG, " send ok");
            } else {
                ESP_LOGI(TAG, " send error: %x", sendStatus);
            }
        }

        // RECEIVE ON CAN0 //
        uint8_t recvStatus;        
        if(gpio_get_level(CAN0_INT_pin) == 0) {  // we received some data
            ESP_LOGI(TAG, "receiving...");
            recvStatus = CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
            if(recvStatus != CAN_OK) {
                ESP_LOGI(TAG, " no message");
            } else {
                if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
                    sprintf(rxString, "Extended ID: 0x%.8lX  DLC: %1d  Received:", (rxId & 0x1FFFFFFF), rxLen);
                else
                    sprintf(rxString, "Standard ID: 0x%.3lX       DLC: %1d  Received:", rxId, rxLen);
                //ESP_LOGI(TAG, "%s", rxString);

                if((rxId & 0x40000000) == 0x40000000) {    // Determine if message is a remote request frame.
                    sprintf(rxString+strlen(rxString), " REMOTE REQUEST FRAME");
                } else {
                    uint8_t pass = 1;
                    for(uint8_t i = 0; i<rxLen; i++){
                        sprintf(rxString+strlen(rxString), " %.2X", rxBuf[i]);
                        if(rxBuf[i] != txBuf[i]) {
                            pass = 0;
                        }
                    }
                    if(pass) {
                        sprintf(rxString+strlen(rxString), "   PASS");
                    } else {
                        sprintf(rxString+strlen(rxString), "   FAIL");
                    }
                }
                ESP_LOGI(TAG, "%s", rxString);
            }
        } 
    }
}

// ESP-IDF says app_main() must be C only, so keep the CPP elsewhere
extern "C" void app_main(void)
{
    setup();
    the_real_main();
}
