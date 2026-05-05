#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"
#include <RadioLib.h>
#include "../remote/EspHal.h"

// ─────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────

#define LORA_CS_PIN 4
#define LORA_DIO0_PIN 7
#define LORA_RST_PIN RADIOLIB_NC

#define MOTOR_AIN1 GPIO_NUM_15
#define MOTOR_AIN2 GPIO_NUM_32
#define MOTOR_PWMA GPIO_NUM_14

#define ENCODER_PHA GPIO_NUM_22
#define ENCODER_PHB GPIO_NUM_20

// ─────────────────────────────────────────────
// LORA SETTINGS
// ─────────────────────────────────────────────

#define LORA_FREQ 915.0
#define LORA_BW 125.0
#define LORA_SF 12
#define LORA_CR 8
#define LORA_SYNC 0x12

// ─────────────────────────────────────────────
// MOTOR / ENCODER SETTINGS
// ─────────────────────────────────────────────

#define TICKS_PER_REV 7000 * 2
#define TICKS_PER_30_DEG (TICKS_PER_REV / 11)

static volatile int home_offset = 0;
static volatile int next_count = 0;

// ─────────────────────────────────────────────
// GLOBALS
// ─────────────────────────────────────────────

static EspHal *hal = new EspHal(5, 21, 19); // SCK=5, MISO=21, MOSI=19
static SX1276 radio = new Module(hal, LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, RADIOLIB_NC);

static pcnt_unit_handle_t pcnt_unit = NULL;
static volatile int target_position = 0;

// ─────────────────────────────────────────────
// MOTOR CONTROL
// ─────────────────────────────────────────────

static void set_motor_speed(int speed)
{
    if (speed > 1023)
        speed = 1023;
    if (speed < -1023)
        speed = -1023;

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
        // Hard brake
        gpio_set_level(MOTOR_AIN1, 1);
        gpio_set_level(MOTOR_AIN2, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ─────────────────────────────────────────────
// PID TASK
// ─────────────────────────────────────────────

static void motor_pid_task(void *arg)
{
    const float Kp = 2.5f;
    int current_pos = 0;

    while (1)
    {
        pcnt_unit_get_count(pcnt_unit, &current_pos);

        int combined_target = home_offset + (next_count * TICKS_PER_30_DEG);
        int error = combined_target - current_pos;
        // printf("POS: %d | TARGET: %d | ERR: %d\n", current_pos, target_position, error);

        // printf("POS: %d | PHA: %d | PHB: %d | TARGET: %d\n",
        //        current_pos,
        //        gpio_get_level(ENCODER_PHA),
        //        gpio_get_level(ENCODER_PHB),
        //        target_position);
        if (abs(error) < 5)
        {
            set_motor_speed(0);
        }
        else
        {
            int output = (int)(Kp * error);
            if (output > 1023)
                output = 1023;
            if (output < -1023)
                output = -1023;
            if (output > 0 && output < 150)
                output = 150;
            if (output < 0 && output > -150)
                output = -150;
            set_motor_speed(output);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void pot_task(void *arg)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &chan_cfg); // GPIO 39

    while (1)
    {
        int raw = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &raw);
        // Map 0-4095 to 0-100
        int pot_percent = (raw * 100) / 4095;
        // Map 0-100% to 0 - TICKS_PER_REV
        home_offset = (pot_percent * TICKS_PER_REV) / 100;
        printf("POT: %d%% | HOME OFFSET: %d ticks\n", pot_percent, home_offset);
        vTaskDelay(pdMS_TO_TICKS(50));
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

        if (state == RADIOLIB_ERR_NONE)
        {
            printf("RX: '%s' | RSSI: %.1f dBm | SNR: %.1f dB\n",
                   (char *)buf, radio.getRSSI(), radio.getSNR());

            if (strncmp((char *)buf, "NEXT", 4) == 0)
            {
                printf("--> NEXT received, indexing 30 degrees\n");

                // ─── NEW ACK CODE ───
                printf("Sending ACK back to remote...\n");
                int tx_state = radio.transmit("ACK");
                if (tx_state != RADIOLIB_ERR_NONE)
                {
                    printf("Failed to send ACK, error: %d\n", tx_state);
                }
                // ────────────────────

                next_count++;
            }
        }
        else if (state == RADIOLIB_ERR_CRC_MISMATCH)
        {
            // printf("RX: CRC error\n");
        }
        else if (state != RADIOLIB_ERR_RX_TIMEOUT)
        {
            // printf("RX error: %d\n", state);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─────────────────────────────────────────────
// INIT — called from main.cpp
// ─────────────────────────────────────────────

void v1_motor_init(void)
{
    if (pcnt_unit != NULL)
    {
        pcnt_unit_stop(pcnt_unit);
        pcnt_unit_disable(pcnt_unit);
        pcnt_del_unit(pcnt_unit);
        pcnt_unit = NULL;
    }
    {

        printf("V1 Motor Carrier starting...\n");

        // ── Motor direction pins ──
        gpio_config_t dir_conf = {};
        dir_conf.pin_bit_mask = (1ULL << MOTOR_AIN1) | (1ULL << MOTOR_AIN2);
        dir_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&dir_conf);
        gpio_set_direction(MOTOR_AIN1, GPIO_MODE_OUTPUT);
        gpio_set_direction(MOTOR_AIN2, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_AIN1, 0);
        gpio_set_level(MOTOR_AIN2, 0);

        // ── PWM ──
        ledc_timer_config_t ledc_timer = {};
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
        ledc_timer.timer_num = LEDC_TIMER_0;
        ledc_timer.freq_hz = 20000;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);

        ledc_channel_config_t ledc_ch = {};
        ledc_ch.gpio_num = MOTOR_PWMA;
        ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_ch.channel = LEDC_CHANNEL_0;
        ledc_ch.timer_sel = LEDC_TIMER_0;
        ledc_ch.duty = 0;
        ledc_ch.hpoint = 0;
        ledc_channel_config(&ledc_ch);

        // ── Encoder PCNT ──
        pcnt_unit_config_t unit_cfg = {};
        unit_cfg.low_limit = -32000;
        unit_cfg.high_limit = 32000;
        pcnt_new_unit(&unit_cfg, &pcnt_unit);

        // Channel A: count on PHA edges, direction from PHB
        pcnt_chan_config_t chan_a_cfg = {};
        chan_a_cfg.edge_gpio_num = ENCODER_PHA;
        chan_a_cfg.level_gpio_num = ENCODER_PHB;
        pcnt_channel_handle_t pcnt_chan_a = NULL;
        pcnt_new_channel(pcnt_unit, &chan_a_cfg, &pcnt_chan_a);
        // Channel A: increase on rising, hold on falling
        pcnt_channel_set_edge_action(pcnt_chan_a,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE, // rising edge
                                     PCNT_CHANNEL_EDGE_ACTION_HOLD);    // falling edge
        // B level controls direction
        pcnt_channel_set_level_action(pcnt_chan_a,
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // B=0 → keep direction
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE); // B=1 → reverse direction

        // Channel B: count on PHB edges, direction from PHA
        pcnt_chan_config_t chan_b_cfg = {};
        chan_b_cfg.edge_gpio_num = ENCODER_PHB;
        chan_b_cfg.level_gpio_num = ENCODER_PHA;
        pcnt_channel_handle_t pcnt_chan_b = NULL;
        pcnt_new_channel(pcnt_unit, &chan_b_cfg, &pcnt_chan_b);
        // Channel B: increase on rising, hold on falling
        pcnt_channel_set_edge_action(pcnt_chan_b,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_HOLD);
        // A level controls direction
        pcnt_channel_set_level_action(pcnt_chan_b,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE, // A=0 → reverse
                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP);   // A=1 → keep
        pcnt_unit_enable(pcnt_unit);
        pcnt_unit_clear_count(pcnt_unit);
        pcnt_unit_start(pcnt_unit);
        gpio_set_pull_mode((gpio_num_t)ENCODER_PHA, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode((gpio_num_t)ENCODER_PHB, GPIO_PULLUP_ONLY);
        printf("Encoder PCNT ready on GPIO%d / GPIO%d\n", ENCODER_PHA, ENCODER_PHB);

        // ── LoRa ──
        int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, 20);
        if (state != RADIOLIB_ERR_NONE)
        {
            printf("LoRa init failed: %d\n", state);
            while (1)
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
        printf("LoRa ready\n");

        // ── Tasks ──
        xTaskCreate(motor_pid_task, "motor_pid", 4096, NULL, 5, NULL);
        xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);
        xTaskCreate(pot_task, "pot", 2048, NULL, 3, NULL);

        // Keep init function alive (required if called from app_main directly)
        while (1)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// #include <stdio.h>
// #include <string.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/ledc.h"
// #include "driver/pulse_cnt.h"
// #include "esp_adc/adc_oneshot.h"
// #include <RadioLib.h>
// #include "../remote/EspHal.h"

// // ─────────────────────────────────────────────
// // PIN DEFINITIONS (V1 Single Motor Circuit)
// // ─────────────────────────────────────────────

// #define LORA_CS_PIN 4
// #define LORA_DIO0_PIN 7
// #define LORA_RST_PIN RADIOLIB_NC

// #define MOTOR_AIN1 GPIO_NUM_15
// #define MOTOR_AIN2 GPIO_NUM_32
// #define MOTOR_PWMA GPIO_NUM_14

// #define ENCODER_PHA GPIO_NUM_22
// #define ENCODER_PHB GPIO_NUM_20

// // ─────────────────────────────────────────────
// // LORA SETTINGS
// // ─────────────────────────────────────────────

// #define LORA_FREQ 915.0
// #define LORA_BW 125.0
// #define LORA_SF 12
// #define LORA_CR 8
// #define LORA_SYNC 0x12

// // ─────────────────────────────────────────────
// // MOTOR / ENCODER SETTINGS
// // ─────────────────────────────────────────────

// #define TICKS_PER_REV (7000 * 2)
// #define TICKS_70_DEG (int)((TICKS_PER_REV / 360.0) * 70.0)

// // ─────────────────────────────────────────────
// // GLOBALS
// // ─────────────────────────────────────────────

// static EspHal *hal = new EspHal(5, 21, 19);
// static SX1276 radio = new Module(hal, LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, RADIOLIB_NC);

// static pcnt_unit_handle_t pcnt_unit = NULL;

// static volatile int home_offset = 0;     // Controlled by the Potentiometer
// static volatile int target_position = 0; // Controlled by the Sequence (0 or 70 deg)

// static volatile bool trigger_next = false;
// static volatile bool sequence_active = false;

// // ─────────────────────────────────────────────
// // MOTOR CONTROL
// // ─────────────────────────────────────────────

// static void set_motor_speed(int speed)
// {
//     if (speed > 1023)
//         speed = 1023;
//     if (speed < -1023)
//         speed = -1023;

//     if (speed > 0)
//     {
//         gpio_set_level(MOTOR_AIN1, 1);
//         gpio_set_level(MOTOR_AIN2, 0);
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
//     }
//     else if (speed < 0)
//     {
//         gpio_set_level(MOTOR_AIN1, 0);
//         gpio_set_level(MOTOR_AIN2, 1);
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, -speed);
//     }
//     else
//     {
//         // Hard brake
//         gpio_set_level(MOTOR_AIN1, 1);
//         gpio_set_level(MOTOR_AIN2, 1);
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
//     }
//     ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
// }

// // ─────────────────────────────────────────────
// // TASKS
// // ─────────────────────────────────────────────

// static void pot_task(void *arg)
// {
//     adc_oneshot_unit_handle_t adc_handle;
//     adc_oneshot_unit_init_cfg_t unit_cfg = {};
//     unit_cfg.unit_id = ADC_UNIT_1;
//     unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
//     adc_oneshot_new_unit(&unit_cfg, &adc_handle);

//     adc_oneshot_chan_cfg_t chan_cfg = {};
//     chan_cfg.atten = ADC_ATTEN_DB_12;
//     chan_cfg.bitwidth = ADC_BITWIDTH_12;
//     adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &chan_cfg); // GPIO 39

//     while (1)
//     {
//         int raw = 0;
//         adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &raw);
//         // Map 0-4095 to 0-100%
//         int pot_percent = (raw * 100) / 4095;
//         // Map 0-100% to 0 - 1 Full Revolution
//         home_offset = (pot_percent * TICKS_PER_REV) / 100;

//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

// static void motor_pid_task(void *arg)
// {
//     const float Kp = 2.5f;
//     int current_pos = 0;

//     while (1)
//     {
//         pcnt_unit_get_count(pcnt_unit, &current_pos);

//         // Master Target = Where the knob is + where the sequence wants to be
//         int combined_target = home_offset + target_position;
//         int error = combined_target - current_pos;

//         if (abs(error) < 5)
//         {
//             set_motor_speed(0);
//         }
//         else
//         {
//             int output = (int)(Kp * error);
//             if (output > 1023)
//                 output = 1023;
//             if (output < -1023)
//                 output = -1023;
//             if (output > 0 && output < 150)
//                 output = 150;
//             if (output < 0 && output > -150)
//                 output = -150;

//             set_motor_speed(output);
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// static void payload_sequence_task(void *arg)
// {
//     while (1)
//     {
//         if (trigger_next)
//         {
//             trigger_next = false;
//             sequence_active = true;
//             printf("--> Opening lid (Target: +%d ticks from home)\n", TICKS_70_DEG);

//             // 1. OPEN LID
//             target_position = TICKS_70_DEG;
//             int timeout = 0;
//             int current_pos = 0;
//             while (timeout++ < 200)
//             {
//                 pcnt_unit_get_count(pcnt_unit, &current_pos);
//                 // Check against the combined target so the timeout doesn't hang if you move the knob
//                 int combined_target = home_offset + target_position;
//                 if (abs(combined_target - current_pos) < 15)
//                     break;
//                 vTaskDelay(pdMS_TO_TICKS(10));
//             }

//             // 2. WAIT
//             printf("--> Lid open. Waiting 1 sec.\n");
//             vTaskDelay(pdMS_TO_TICKS(1000));

//             // 3. CLOSE LID
//             printf("--> Closing lid (Target: back to home)\n");
//             target_position = 0;
//             timeout = 0;
//             while (timeout++ < 200)
//             {
//                 pcnt_unit_get_count(pcnt_unit, &current_pos);
//                 int combined_target = home_offset + target_position;
//                 if (abs(combined_target - current_pos) < 15)
//                     break;
//                 vTaskDelay(pdMS_TO_TICKS(10));
//             }

//             set_motor_speed(0); // Ensure hard brake
//             sequence_active = false;
//             printf("--> Sequence Complete. Ready for next command.\n");
//         }
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// static void lora_rx_task(void *arg)
// {
//     printf("LoRa RX standing by...\n");

//     while (1)
//     {
//         uint8_t buf[16] = {0};
//         int state = radio.receive(buf, sizeof(buf));

//         if (state == RADIOLIB_ERR_NONE)
//         {
//             printf("RX: '%s' | RSSI: %.1f dBm | SNR: %.1f dB\n",
//                    (char *)buf, radio.getRSSI(), radio.getSNR());

//             if (strncmp((char *)buf, "NEXT", 4) == 0)
//             {
//                 // Send ACK immediately
//                 printf("--> NEXT received. Sending ACK back to remote...\n");
//                 int tx_state = radio.transmit("ACK");
//                 if (tx_state != RADIOLIB_ERR_NONE)
//                 {
//                     printf("Failed to send ACK, error: %d\n", tx_state);
//                 }

//                 // Trigger sequence if not already running
//                 if (!sequence_active)
//                 {
//                     trigger_next = true;
//                 }
//                 else
//                 {
//                     printf("--> NEXT ignored (Sequence already in progress)\n");
//                 }
//             }
//         }
//         else if (state != RADIOLIB_ERR_RX_TIMEOUT && state != RADIOLIB_ERR_CRC_MISMATCH)
//         {
//             // Only print serious radio hardware errors
//             printf("RX error: %d\n", state);
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// // ─────────────────────────────────────────────
// // INIT
// // ─────────────────────────────────────────────

// void v1_motor_init(void)
// {
//     if (pcnt_unit != NULL)
//     {
//         pcnt_unit_stop(pcnt_unit);
//         pcnt_unit_disable(pcnt_unit);
//         pcnt_del_unit(pcnt_unit);
//         pcnt_unit = NULL;
//     }

//     printf("V1 Single-Motor Test (With Potentiometer) starting...\n");

//     // ── Motor direction pins ──
//     gpio_config_t dir_conf = {};
//     dir_conf.pin_bit_mask = (1ULL << MOTOR_AIN1) | (1ULL << MOTOR_AIN2);
//     dir_conf.mode = GPIO_MODE_OUTPUT;
//     gpio_config(&dir_conf);
//     gpio_set_level(MOTOR_AIN1, 0);
//     gpio_set_level(MOTOR_AIN2, 0);

//     // ── PWM ──
//     ledc_timer_config_t ledc_timer = {};
//     ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
//     ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
//     ledc_timer.timer_num = LEDC_TIMER_0;
//     ledc_timer.freq_hz = 20000;
//     ledc_timer.clk_cfg = LEDC_AUTO_CLK;
//     ledc_timer_config(&ledc_timer);

//     ledc_channel_config_t ledc_ch = {};
//     ledc_ch.gpio_num = MOTOR_PWMA;
//     ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
//     ledc_ch.channel = LEDC_CHANNEL_0;
//     ledc_ch.timer_sel = LEDC_TIMER_0;
//     ledc_ch.duty = 0;
//     ledc_ch.hpoint = 0;
//     ledc_channel_config(&ledc_ch);

//     // ── Encoder PCNT ──
//     pcnt_unit_config_t unit_cfg = {};
//     unit_cfg.low_limit = -32000;
//     unit_cfg.high_limit = 32000;
//     pcnt_new_unit(&unit_cfg, &pcnt_unit);

//     pcnt_chan_config_t chan_a_cfg = {};
//     chan_a_cfg.edge_gpio_num = ENCODER_PHA;
//     chan_a_cfg.level_gpio_num = ENCODER_PHB;
//     pcnt_channel_handle_t pcnt_chan_a = NULL;
//     pcnt_new_channel(pcnt_unit, &chan_a_cfg, &pcnt_chan_a);
//     pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
//     pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

//     pcnt_chan_config_t chan_b_cfg = {};
//     chan_b_cfg.edge_gpio_num = ENCODER_PHB;
//     chan_b_cfg.level_gpio_num = ENCODER_PHA;
//     pcnt_channel_handle_t pcnt_chan_b = NULL;
//     pcnt_new_channel(pcnt_unit, &chan_b_cfg, &pcnt_chan_b);
//     pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
//     pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

//     pcnt_unit_enable(pcnt_unit);
//     pcnt_unit_clear_count(pcnt_unit);
//     pcnt_unit_start(pcnt_unit);
//     gpio_set_pull_mode((gpio_num_t)ENCODER_PHA, GPIO_PULLUP_ONLY);
//     gpio_set_pull_mode((gpio_num_t)ENCODER_PHB, GPIO_PULLUP_ONLY);

//     // ── LoRa ──
//     int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, 20);
//     if (state != RADIOLIB_ERR_NONE)
//     {
//         printf("LoRa init failed: %d\n", state);
//         while (1)
//             vTaskDelay(pdMS_TO_TICKS(1000));
//     }
//     printf("LoRa ready\n");

//     // ── Tasks ──
//     xTaskCreate(pot_task, "pot", 2048, NULL, 3, NULL);
//     xTaskCreate(motor_pid_task, "motor_pid", 4096, NULL, 5, NULL);
//     xTaskCreate(payload_sequence_task, "sequence", 4096, NULL, 4, NULL);
//     xTaskCreate(lora_rx_task, "lora_rx", 4096, NULL, 5, NULL);

//     while (1)
//         vTaskDelay(pdMS_TO_TICKS(1000));
// }