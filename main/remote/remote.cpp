#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include <RadioLib.h>
#include "EspHal.h"

#define LORA_CS_PIN 4
#define LORA_DIO0_PIN 7
#define LORA_RST_PIN RADIOLIB_NC

#define BUTTON_PIN 14

#define LED_GREEN 32
#define LED_BLUE 15
#define LED_RED 33

#define LORA_FREQ 915.0
#define LORA_BW 125.0
#define LORA_SF 12
#define LORA_CR 8
#define LORA_SYNC 0x12
#define LORA_TX_POWER 20

EspHal *hal = new EspHal(5, 21, 19);
static SX1276 radio = new Module(hal, LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, RADIOLIB_NC);

// ─────────────────────────────────────────────
// LED HELPERS
// ─────────────────────────────────────────────
static void init_leds()
{
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GREEN) | (1ULL << LED_BLUE) | (1ULL << LED_RED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);

    // Ensure all are off initially
    gpio_set_level((gpio_num_t)LED_GREEN, 0);
    gpio_set_level((gpio_num_t)LED_BLUE, 0);
    gpio_set_level((gpio_num_t)LED_RED, 0);
}

static void blink_led(int pin, int blinks)
{
    for (int i = 0; i < blinks; i++)
    {
        gpio_set_level((gpio_num_t)pin, 1);
        vTaskDelay(pdMS_TO_TICKS(80));
        gpio_set_level((gpio_num_t)pin, 0);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

// ─────────────────────────────────────────────
// BUTTON TASK
// ─────────────────────────────────────────────
static void button_task(void *arg)
{
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    int last_state = 1;

    while (1)
    {
        int state = gpio_get_level((gpio_num_t)BUTTON_PIN);

        if (last_state == 1 && state == 0)
        {
            printf("Button pressed — sending NEXT\n");

            // 1. Turn Blue LED ON (Status: Working)
            gpio_set_level((gpio_num_t)LED_BLUE, 1);

            // 2. Transmit the command
            int result = radio.transmit("NEXT");

            if (result == RADIOLIB_ERR_NONE)
            {
                printf("TX OK. Waiting for ACK...\n");

                // 3. Switch to Receive mode to listen for ACK
                bool ack_received = false;
                radio.startReceive();

                // 2000ms (2 second) timeout window
                TickType_t start_time = xTaskGetTickCount();
                while (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(2000))
                {
                    // Check if LoRa module received a packet (DIO0 goes HIGH)
                    if (gpio_get_level((gpio_num_t)LORA_DIO0_PIN) == 1)
                    {
                        uint8_t rx_buf[16] = {0};
                        int rx_state = radio.readData(rx_buf, sizeof(rx_buf));

                        // Verify it's an ACK and not random noise
                        if (rx_state == RADIOLIB_ERR_NONE && strncmp((char *)rx_buf, "ACK", 3) == 0)
                        {
                            ack_received = true;
                            break;
                        }
                        radio.startReceive(); // Go back to listening if bad packet
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                // 4. Turn Blue OFF, process result
                gpio_set_level((gpio_num_t)LED_BLUE, 0);

                if (ack_received)
                {
                    printf("ACK Received! Payload deployed.\n");
                    blink_led(LED_GREEN, 4); // Fast blink Green
                }
                else
                {
                    printf("No ACK received (Timeout).\n");
                    blink_led(LED_RED, 4); // Fast blink Red
                }
            }
            else
            {
                // TX Hardware Failed
                printf("Send failed, error: %d\n", result);
                gpio_set_level((gpio_num_t)LED_BLUE, 0);
                blink_led(LED_RED, 4);
            }

            vTaskDelay(pdMS_TO_TICKS(500)); // Debounce and cooldown
        }

        last_state = state;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────
void remote_init(void)
{
    printf("LoRa remote starting...\n");

    init_leds();

    // Quick boot sequence to test LEDs
    blink_led(LED_BLUE, 1);
    blink_led(LED_GREEN, 1);
    blink_led(LED_RED, 1);

    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, LORA_TX_POWER);

    if (state != RADIOLIB_ERR_NONE)
    {
        printf("LoRa init failed, error: %d\n", state);
        while (1)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("LoRa init OK. Press button on GPIO14 to send NEXT\n");
    xTaskCreate(button_task, "button", 4096, NULL, 5, NULL);
}