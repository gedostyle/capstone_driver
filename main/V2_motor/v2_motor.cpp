#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include <RadioLib.h>
#include "../remote/EspHal.h"

// ─────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────

// LoRa
#define LORA_CS_PIN 4
#define LORA_DIO0_PIN 7
#define LORA_RST_PIN RADIOLIB_NC

// Shift Register
#define SR_DATA_PIN GPIO_NUM_8   // SER
#define SR_LATCH_PIN GPIO_NUM_25 // RCLK
#define SR_CLOCK_PIN GPIO_NUM_26 // SRCLK

// Encoders (via MUX)
#define ENCODER_PHA GPIO_NUM_22 // C1
#define ENCODER_PHB GPIO_NUM_20 // C2

// Motor Driver Outputs
#define MOTOR_AIN1 GPIO_NUM_15
#define MOTOR_AIN2 GPIO_NUM_32
#define MOTOR_PWMA GPIO_NUM_14
#define MOTOR_BIN1 GPIO_NUM_33
#define MOTOR_BIN2 GPIO_NUM_27
#define MOTOR_PWMB GPIO_NUM_13

// ─────────────────────────────────────────────
// LORA SETTINGS
// ─────────────────────────────────────────────
#define LORA_FREQ 915.0
#define LORA_BW 125.0
#define LORA_SF 12
#define LORA_CR 8
#define LORA_SYNC 0x12

// ─────────────────────────────────────────────
// MOTOR & PAYLOAD SETTINGS
// ─────────────────────────────────────────────
#define TOTAL_MOTORS 14
#define TICKS_PER_REV (7000 * 2)
// Standard math for 70 degrees based on your 14000 tick revolution:
#define TICKS_70_DEG (int)((TICKS_PER_REV / 360.0) * 70.0)

// ─────────────────────────────────────────────
// GLOBALS
// ─────────────────────────────────────────────
static EspHal *hal = new EspHal(5, 21, 19);
static SX1276 radio = new Module(hal, LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, RADIOLIB_NC);

static pcnt_unit_handle_t pcnt_unit = NULL;

static volatile int active_motor_index = 0; // 0 to 13
static volatile int target_position = 0;
static volatile bool sequence_active = false;
static volatile bool trigger_next = false;

// 1 = Normal, -1 = Reversed. Populated by auto-calibrate.
static int motor_polarity[TOTAL_MOTORS];

// ─────────────────────────────────────────────
// HARDWARE ROUTINES
// ─────────────────────────────────────────────

static void shift_out_16(uint16_t data)
{
    gpio_set_level(SR_LATCH_PIN, 0);
    for (int i = 0; i < 16; i++)
    {
        bool bit = (data >> (15 - i)) & 1;
        gpio_set_level(SR_DATA_PIN, bit);
        gpio_set_level(SR_CLOCK_PIN, 0);
        esp_rom_delay_us(1);
        gpio_set_level(SR_CLOCK_PIN, 1);
        esp_rom_delay_us(1);
    }
    gpio_set_level(SR_LATCH_PIN, 1);
}

static void select_motor(int motor_index)
{
    int driver_index = motor_index / 2; // Drivers 0 to 6
    int mux_channel = motor_index;      // Mux 0 to 13

    uint16_t shift_word = 0;
    // Bits 0-6: Driver STBY lines
    shift_word |= (1 << driver_index);
    // Bits 8-11: MUX S0-S3 lines
    shift_word |= (mux_channel << 8);

    shift_out_16(shift_word);

    // Give MUX voltage a microsecond to settle, then clear encoder
    esp_rom_delay_us(50);
    pcnt_unit_clear_count(pcnt_unit);
}

static void set_motor_speed(int motor_index, int speed)
{
    // Apply polarity correction detected during boot
    speed *= motor_polarity[motor_index];

    if (speed > 1023)
        speed = 1023;
    if (speed < -1023)
        speed = -1023;

    bool is_channel_b = (motor_index % 2) != 0;

    if (is_channel_b)
    {
        if (speed > 0)
        {
            gpio_set_level(MOTOR_BIN1, 1);
            gpio_set_level(MOTOR_BIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
        }
        else if (speed < 0)
        {
            gpio_set_level(MOTOR_BIN1, 0);
            gpio_set_level(MOTOR_BIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, -speed);
        }
        else
        {
            gpio_set_level(MOTOR_BIN1, 1);
            gpio_set_level(MOTOR_BIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
    else // Channel A
    {
        if (speed > 0)
        {
            gpio_set_level(MOTOR_AIN1, 1);
            gpio_set_level(MOTOR_AIN2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
        }
        else if (speed < 0)
        {
            gpio_set_level(MOTOR_AIN1, 0);
            gpio_set_level(MOTOR_AIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, -speed);
        }
        else
        {
            gpio_set_level(MOTOR_AIN1, 1);
            gpio_set_level(MOTOR_AIN2, 1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

// ─────────────────────────────────────────────
// CALIBRATION & CONTROL TASKS
// ─────────────────────────────────────────────

static void calibrate_motors()
{
    printf("Starting Auto-Calibration for 14 mechanisms...\n");
    for (int i = 0; i < TOTAL_MOTORS; i++)
    {
        motor_polarity[i] = 1; // Default
        select_motor(i);
        vTaskDelay(pdMS_TO_TICKS(50));

        // Test Pulse Forward
        set_motor_speed(i, 400);
        vTaskDelay(pdMS_TO_TICKS(150));
        set_motor_speed(i, 0);

        int count = 0;
        pcnt_unit_get_count(pcnt_unit, &count);

        if (count < -5)
        {
            motor_polarity[i] = -1;
            printf("Motor %d: REVERSED (Count: %d). Polarity flipped.\n", i, count);
        }
        else if (count > 5)
        {
            printf("Motor %d: NORMAL (Count: %d).\n", i, count);
        }
        else
        {
            printf("Motor %d: NO MOVEMENT DETECTED. Check wiring.\n", i);
        }

        // Return to start using newly corrected polarity
        set_motor_speed(i, -400);
        vTaskDelay(pdMS_TO_TICKS(150));
        set_motor_speed(i, 0);
    }
    printf("Calibration Complete.\n");

    // Reset to start state
    active_motor_index = 0;
    select_motor(active_motor_index);
}

static void motor_pid_task(void *arg)
{
    const float Kp = 2.5f; // Restored from V1
    int current_pos = 0;

    while (1)
    {
        pcnt_unit_get_count(pcnt_unit, &current_pos);
        int error = target_position - current_pos;

        // Restored < 5 tolerance from V1
        if (abs(error) < 5)
        {
            set_motor_speed(active_motor_index, 0);
        }
        else
        {
            int output = (int)(Kp * error);
            if (output > 1023)
                output = 1023;
            if (output < -1023)
                output = -1023;

            // Restored 150 minimum floor from V1
            if (output > 0 && output < 150)
                output = 150;
            if (output < 0 && output > -150)
                output = -150;

            set_motor_speed(active_motor_index, output);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void payload_sequence_task(void *arg)
{
    while (1)
    {
        if (trigger_next)
        {
            trigger_next = false;
            sequence_active = true;
            printf("--> Opening lid %d\n", active_motor_index);

            // 1. OPEN LID
            target_position = TICKS_70_DEG;
            int timeout = 0;
            int current_pos = 0;
            while (timeout++ < 200)
            { // 2 second max timeout
                pcnt_unit_get_count(pcnt_unit, &current_pos);
                if (abs(target_position - current_pos) < 15)
                    break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // 2. WAIT
            printf("--> Lid %d open. Waiting 1 sec.\n", active_motor_index);
            vTaskDelay(pdMS_TO_TICKS(1000));

            // 3. CLOSE LID
            printf("--> Closing lid %d\n", active_motor_index);
            target_position = 0;
            timeout = 0;
            while (timeout++ < 200)
            {
                pcnt_unit_get_count(pcnt_unit, &current_pos);
                if (abs(target_position - current_pos) < 15)
                    break;
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // Ensure hard brake before disabling driver
            set_motor_speed(active_motor_index, 0);

            // 4. INDEX TO NEXT MOTOR
            active_motor_index++;
            if (active_motor_index >= TOTAL_MOTORS)
            {
                active_motor_index = 0; // Loop back to start
                printf("--> Payload Cycle Complete. Looping to 0.\n");
            }

            select_motor(active_motor_index);
            sequence_active = false;
            printf("--> System ready for next command.\n");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─────────────────────────────────────────────
// LORA RX TASK
// ─────────────────────────────────────────────

static void lora_rx_task(void *arg)
{
    printf("LoRa RX standing by...\n");

    while (1)
    {
        uint8_t buf[16] = {0};
        int state = radio.receive(buf, sizeof(buf));

        if (strncmp((char *)buf, "NEXT", 4) == 0)
        {
            printf("Sending ACK back to remote...\n");
            radio.transmit("ACK");

            if (!sequence_active)
            {
                trigger_next = true;
            }
            else
            {
                printf("--> NEXT ignored (Sequence in progress)\n");
            }
        }
        radio.startReceive();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─────────────────────────────────────────────
// INIT
// ─────────────────────────────────────────────

void v2_motor_init(void)
{
    // ── Shift Register Init ──
    gpio_config_t sr_conf = {};
    sr_conf.pin_bit_mask = (1ULL << SR_DATA_PIN) | (1ULL << SR_LATCH_PIN) | (1ULL << SR_CLOCK_PIN);
    sr_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&sr_conf);
    shift_out_16(0x0000); // Clear all

    // ── Motor Driver Pins ──
    gpio_config_t dir_conf = {};
    dir_conf.pin_bit_mask = (1ULL << MOTOR_AIN1) | (1ULL << MOTOR_AIN2) |
                            (1ULL << MOTOR_BIN1) | (1ULL << MOTOR_BIN2);
    dir_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&dir_conf);

    // ── PWM Config (Two Channels Now) ──
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.freq_hz = 20000;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_ch_a = {};
    ledc_ch_a.gpio_num = MOTOR_PWMA;
    ledc_ch_a.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_ch_a.channel = LEDC_CHANNEL_0;
    ledc_ch_a.timer_sel = LEDC_TIMER_0;
    ledc_ch_a.duty = 0;
    ledc_channel_config(&ledc_ch_a);

    ledc_channel_config_t ledc_ch_b = {};
    ledc_ch_b.gpio_num = MOTOR_PWMB;
    ledc_ch_b.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_ch_b.channel = LEDC_CHANNEL_1;
    ledc_ch_b.timer_sel = LEDC_TIMER_0;
    ledc_ch_b.duty = 0;
    ledc_channel_config(&ledc_ch_b);

    // ── Encoder PCNT ──
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit = -32000;
    unit_cfg.high_limit = 32000;
    pcnt_new_unit(&unit_cfg, &pcnt_unit);

    pcnt_chan_config_t chan_a_cfg = {};
    chan_a_cfg.edge_gpio_num = ENCODER_PHA;
    chan_a_cfg.level_gpio_num = ENCODER_PHB;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_cfg, &pcnt_chan_a);
    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_chan_config_t chan_b_cfg = {};
    chan_b_cfg.edge_gpio_num = ENCODER_PHB;
    chan_b_cfg.level_gpio_num = ENCODER_PHA;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_new_channel(pcnt_unit, &chan_b_cfg, &pcnt_chan_b);
    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
    gpio_set_pull_mode(ENCODER_PHA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENCODER_PHB, GPIO_PULLUP_ONLY);

    // ── LoRa ──
    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, 20);
    if (state != RADIOLIB_ERR_NONE)
    {
        printf("LoRa init failed: %d\n", state);
    }

    // ── Boot Sequence ──
    calibrate_motors();

    // ── Tasks ──
    xTaskCreate(motor_pid_task, "motor_pid", 4096, NULL, 5, NULL);
    xTaskCreate(payload_sequence_task, "sequence", 4096, NULL, 4, NULL);
    xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
}