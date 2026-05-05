#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"
#include <esp_timer.h>
#include "driver/pulse_cnt.h"

// ─────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────

// Shift Register
#define DATA_PIN ((gpio_num_t)32)  // SER
#define RCLK_PIN ((gpio_num_t)33)  // LATCH
#define SRCLK_PIN ((gpio_num_t)15) // CLOCK

// Button
#define BUTTON_PIN ((gpio_num_t)4)

// Motor Direction - Channel A (shared across all A channels)
#define AIN1_PIN ((gpio_num_t)25)
#define AIN2_PIN ((gpio_num_t)26)

// Motor Direction - Channel B (shared across all B channels)
#define BIN1_PIN ((gpio_num_t)8)
#define BIN2_PIN ((gpio_num_t)7)

// PWM
#define PWMA_PIN ((gpio_num_t)20)
#define PWMB_PIN ((gpio_num_t)22)

// MUX Select Lines (shared between both MUXes)
#define S0_PIN ((gpio_num_t)14)
#define S1_PIN ((gpio_num_t)27)
#define S2_PIN ((gpio_num_t)12)
#define S3_PIN ((gpio_num_t)13)

// MUX Signal Inputs (read-only)
#define MUX1_SIG ((gpio_num_t)39) // C1 (Encoder Phase A)
#define MUX2_SIG ((gpio_num_t)36) // C2 (Encoder Phase B)

// Potentiometer
#define POT_ADC_UNIT ADC_UNIT_1
#define POT_CHANNEL ADC_CHANNEL_6 // GPIO35 = ADC1_CH5

// ─────────────────────────────────────────────
// CONSTANTS
// ─────────────────────────────────────────────

#define TOTAL_MOTORS 17
#define TOTAL_DRIVERS 9
#define TOTAL_SHIFT_BITS 16

#define MUX1_ENABLE_BIT 9
#define MUX2_ENABLE_BIT 10

// Encoder
#define PULSES_PER_REV 7000
#define TARGET_MAX_PULSES PULSES_PER_REV

// PWM
#define PWM_FREQ_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT // 0-1023
#define PWM_MAX 1023

// Scaled up ~4x from your original 8-bit working code
#define PID_KP 4.8f
#define PID_KI 0.05f
#define PID_KD 1.6f

#define PID_INTEGRAL_LIMIT 400.0f
#define DEAD_ZONE 30.0f // Returned to original tight deadband

#define MUX_SETTLE_US 200

// ─────────────────────────────────────────────
// MOTOR MAPPING
// ─────────────────────────────────────────────
// Motors 0-7  → drivers 0-7, channel A
// Motors 8-15 → drivers 0-7, channel B
// Motor 16    → driver 8,   channel A

typedef enum
{
    CHANNEL_A,
    CHANNEL_B
} motor_channel_t;

typedef struct
{
    int driver_index; // Which TB6612FNG (0-8)
    motor_channel_t channel;
} motor_map_t;

static const motor_map_t MOTOR_MAP[TOTAL_MOTORS] = {
    {0, CHANNEL_A}, // Motor 0
    {0, CHANNEL_B}, // Motor 1
    {1, CHANNEL_A}, // Motor 2
    {1, CHANNEL_B}, // Motor 3
    {2, CHANNEL_A}, // Motor 4
    {2, CHANNEL_B}, // Motor 5
    {3, CHANNEL_A}, // Motor 6
    {3, CHANNEL_B}, // Motor 7
    {4, CHANNEL_A}, // Motor 8
    {4, CHANNEL_B}, // Motor 9
    {5, CHANNEL_A}, // Motor 10
    {5, CHANNEL_B}, // Motor 11
    {6, CHANNEL_A}, // Motor 12
    {6, CHANNEL_B}, // Motor 13
    {7, CHANNEL_A}, // Motor 14
    {7, CHANNEL_B}, // Motor 15
    {8, CHANNEL_A}  // Motor 16
};

// ─────────────────────────────────────────────
// GLOBAL STATE
// ─────────────────────────────────────────────

static volatile int32_t encoder_count = 0; // Current pulse count for active motor
static volatile int last_c1 = 0;           // Last state of C1 for direction
static volatile int last_c2 = 0;           // Last state of C2 for direction
static volatile int32_t pot_target = 0;

static int active_motor = 0; // Which motor is currently selected

static adc_oneshot_unit_handle_t adc_handle; // ADC1 oneshot handle

static SemaphoreHandle_t encoder_mutex;

static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan = NULL;

typedef enum
{
    MANUAL_STOP,
    MANUAL_LEFT,
    MANUAL_RIGHT
} manual_state_t;
static volatile manual_state_t current_mode = MANUAL_STOP;

// ─────────────────────────────────────────────
// SHIFT REGISTER
// ─────────────────────────────────────────────

// Builds the 16-bit word for the shift register.
// Sets the STBY bit for the active driver, keeps MUX enables high.
static uint16_t build_shift_word(int driver_index)
{
    uint16_t word = 0;
    word |= (1 << driver_index); // STBY for active driver
    // word |= (0 << MUX1_ENABLE_BIT); // MUX1 always enabled
    // word |= (0 << MUX2_ENABLE_BIT); // MUX2 always enabled
    return word;
}

static void shift_out_16(uint16_t data)
{
    gpio_set_level(RCLK_PIN, 0);

    for (int i = 0; i < TOTAL_SHIFT_BITS; i++)
    {
        bool bit = (data >> (TOTAL_SHIFT_BITS - 1 - i)) & 1;
        gpio_set_level(DATA_PIN, bit);
        gpio_set_level(SRCLK_PIN, 0);
        esp_rom_delay_us(1);
        gpio_set_level(SRCLK_PIN, 1);
        esp_rom_delay_us(1);
    }

    gpio_set_level(RCLK_PIN, 1);
}

// ─────────────────────────────────────────────
// MUX CHANNEL SELECT
// ─────────────────────────────────────────────

// Motor index maps directly to MUX channel (0-16)
static void set_mux_channel(int motor_index)
{
    gpio_set_level(S0_PIN, (motor_index >> 0) & 1);
    gpio_set_level(S1_PIN, (motor_index >> 1) & 1);
    gpio_set_level(S2_PIN, (motor_index >> 2) & 1);
    gpio_set_level(S3_PIN, (motor_index >> 3) & 1);
    esp_rom_delay_us(MUX_SETTLE_US);
}

// ─────────────────────────────────────────────
// MOTOR DRIVE
// ─────────────────────────────────────────────

static void set_motor(motor_channel_t ch, int duty, bool forward)
{
    // Clamp duty strictly to 0 - 1023
    if (duty < 0)
        duty = 0;
    if (duty > PWM_MAX)
        duty = PWM_MAX;

    if (ch == CHANNEL_A)
    {
        gpio_set_level(AIN1_PIN, forward ? 1 : 0);
        gpio_set_level(AIN2_PIN, forward ? 0 : 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else
    {
        gpio_set_level(BIN1_PIN, forward ? 1 : 0);
        gpio_set_level(BIN2_PIN, forward ? 0 : 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

static void stop_motor(motor_channel_t ch)
{
    if (ch == CHANNEL_A)
    {
        gpio_set_level(AIN1_PIN, 1);
        gpio_set_level(AIN2_PIN, 1); // Short brake
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else
    {
        gpio_set_level(BIN1_PIN, 1);
        gpio_set_level(BIN2_PIN, 1); // Short brake
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

static void adc_task(void *arg)
{
    while (1)
    {
        int raw = 0;
        adc_oneshot_read(adc_handle, POT_CHANNEL, &raw);
        pot_target = (int32_t)((raw * (int64_t)TARGET_MAX_PULSES) / 4095);
        vTaskDelay(pdMS_TO_TICKS(50)); // Read pot at 20Hz, yields properly
    }
}

// Button Debug

static void button_task(void *arg)
{
    int last_state = 1;

    while (1)
    {
        int state = gpio_get_level(BUTTON_PIN);

        // On button press (falling edge)
        if (last_state == 1 && state == 0)
        {
            if (current_mode == MANUAL_STOP)
                current_mode = MANUAL_LEFT;
            else if (current_mode == MANUAL_LEFT)
                current_mode = MANUAL_RIGHT;
            else
                current_mode = MANUAL_STOP;

            vTaskDelay(pdMS_TO_TICKS(200)); // Debounce
        }

        last_state = state;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─────────────────────────────────────────────
// ENCODER INTERRUPTS
// ─────────────────────────────────────────────

// Quadrature decode: interrupt fires on any edge of C1 or C2.
// Direction is determined by comparing the two phases.

// static void IRAM_ATTR encoder_c1_isr(void *arg)
// {
//     static int64_t last_time_c1 = 0;
//     int64_t now = esp_timer_get_time();
//     if (now - last_time_c1 < 500)
//         return; // Ignore pulses faster than 500µs
//     last_time_c1 = now;

//     int c1 = gpio_get_level(MUX1_SIG);
//     int c2 = gpio_get_level(MUX2_SIG);

//     if (c1 == c2)
//     {
//         encoder_count = encoder_count + 1; // Forward
//     }
//     else
//     {
//         encoder_count = encoder_count - 1; // Reverse
//     }
//     last_c1 = c1;
// }

// static void IRAM_ATTR encoder_c2_isr(void *arg)
// {
//     static int64_t last_time_c2 = 0;
//     int64_t now = esp_timer_get_time();
//     if (now - last_time_c2 < 500)
//         return; // Ignore pulses faster than 500µs
//     last_time_c2 = now;

//     int c1 = gpio_get_level(MUX1_SIG);
//     int c2 = gpio_get_level(MUX2_SIG);

//     if (c1 != c2)
//     {
//         encoder_count = encoder_count + 1; // Forward
//     }
//     else
//     {
//         encoder_count = encoder_count - 1; // Reverse
//     }
//     last_c2 = c2;
//     // printf("ISR fired: %ld\n", (long)encoder_count); // temp debug
// }

// ─────────────────────────────────────────────
// ADC / POT
// ─────────────────────────────────────────────
static void init_pcnt(void)
{
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit = -32768;
    unit_cfg.high_limit = 32767;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_unit));

    pcnt_chan_config_t chan_cfg = {};
    chan_cfg.edge_gpio_num = MUX1_SIG;  // GPIO 39
    chan_cfg.level_gpio_num = MUX2_SIG; // GPIO 36
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_cfg, &pcnt_chan));

    // Hardware handles the direction logic instantly
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Optional: Built-in hardware glitch filter (ignores noise under 1000ns)
    pcnt_glitch_filter_config_t filter_cfg = {};
    filter_cfg.max_glitch_ns = 1000;
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg);

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

static int32_t read_pot_target(void)
{
    // Average 4 samples for stability
    int sum = 0;
    for (int i = 0; i < 4; i++)
    {
        int raw = 0;
        adc_oneshot_read(adc_handle, POT_CHANNEL, &raw);
        sum += raw;
    }
    int raw = sum / 4; // 0-4095

    // Map to 0 - PULSES_PER_REV
    return (int32_t)((raw * (int64_t)TARGET_MAX_PULSES) / 4095);
}

// ─────────────────────────────────────────────
// MOTOR SELECTION (button press)
// ─────────────────────────────────────────────

static void select_motor(int motor_index)
{
    motor_channel_t old_ch = MOTOR_MAP[active_motor].channel;
    stop_motor(old_ch);

    active_motor = motor_index;
    int driver = MOTOR_MAP[motor_index].driver_index;

    shift_out_16(build_shift_word(driver));
    set_mux_channel(motor_index);

    // Give the MUX a tiny moment to settle its voltage before clearing
    esp_rom_delay_us(MUX_SETTLE_US);

    // Clear the hardware pulse counter for the new motor
    pcnt_unit_clear_count(pcnt_unit);

    printf("Active motor: %d | Driver: %d | Channel: %s\n",
           motor_index, driver,
           MOTOR_MAP[motor_index].channel == CHANNEL_A ? "A" : "B");
}

// ─────────────────────────────────────────────
// PID TASK
// ─────────────────────────────────────────────

// static void pid_task(void *arg)
// {
//     float integral = 0.0f;
//     float last_error = 0.0f;
//     TickType_t last_time = xTaskGetTickCount();

//     while (1)
//     {
//         TickType_t now = xTaskGetTickCount();
//         float dt = (now - last_time) / (float)configTICK_RATE_HZ;
//         last_time = now;
//         if (dt <= 0.0f)
//             dt = 0.001f;

//         int32_t target = pot_target; // Read from the global set by adc_task

//         // target = -100;
//         // Replace the portDISABLE block with this single line:
//         int current_pos = 0;
//         pcnt_unit_get_count(pcnt_unit, &current_pos);
//         int32_t current = (int32_t)current_pos;
//         float error = (float)(target - current);

//         motor_channel_t ch = MOTOR_MAP[active_motor].channel;

//         if (fabsf(error) < DEAD_ZONE)
//         {
//             stop_motor(ch);
//             integral = 0.0f; // Reset to avoid windup when resting
//         }
//         else
//         {
//             integral += error * PID_KI;

//             // Anti-windup
//             if (integral > PID_INTEGRAL_LIMIT)
//                 integral = PID_INTEGRAL_LIMIT;
//             if (integral < -PID_INTEGRAL_LIMIT)
//                 integral = -PID_INTEGRAL_LIMIT;

//             float derivative = error - last_error;

//             // Standard PID Equation
//             float output = (PID_KP * error) + integral + (PID_KD * derivative);

//             bool forward = (output >= 0);
//             int duty = (int)fabsf(output);

//             set_motor(ch, duty, forward);
//         }

//         last_error = error;

//         // Keep debug print outside the logic loop so it doesn't slow down calculation
//         printf("Target: %ld  Position: %ld  Error: %.1f\n", target, current, error);
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

static void pid_task(void *arg)
{
    while (1)
    {
        // 1. Read hardware encoder to verify position tracking
        int current_pos = 0;
        pcnt_unit_get_count(pcnt_unit, &current_pos);

        motor_channel_t ch = MOTOR_MAP[active_motor].channel;

        // 2. Open-loop manual control overrides
        if (current_mode == MANUAL_STOP)
        {
            stop_motor(ch);
            printf("Mode: STOP  | Position: %d\n", current_pos);
        }
        else if (current_mode == MANUAL_LEFT)
        {
            // 500/1023 is roughly 50% duty cycle. False = reverse.
            set_motor(ch, 500, false);
            printf("Mode: LEFT  | Position: %d\n", current_pos);
        }
        else if (current_mode == MANUAL_RIGHT)
        {
            // 500/1023 is roughly 50% duty cycle. True = forward.
            set_motor(ch, 500, true);
            printf("Mode: RIGHT | Position: %d\n", current_pos);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ─────────────────────────────────────────────
// BUTTON TASK
// ─────────────────────────────────────────────

// static void button_task(void *arg)
// {
//     int last_state = 1;

//     while (1)
//     {
//         int state = gpio_get_level(BUTTON_PIN);

//         if (last_state == 1 && state == 0)
//         {
//             int next = (active_motor + 1) % TOTAL_MOTORS;
//             select_motor(next);
//             vTaskDelay(pdMS_TO_TICKS(200)); // Debounce
//         }

//         last_state = state;
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }

// ─────────────────────────────────────────────
// HARDWARE INIT
// ─────────────────────────────────────────────

static void init_gpio(void)
{
    // Output pins - Zero Initialized!
    gpio_config_t out_conf = {};
    out_conf.pin_bit_mask = (1ULL << DATA_PIN) |
                            (1ULL << RCLK_PIN) |
                            (1ULL << SRCLK_PIN) |
                            (1ULL << AIN1_PIN) |
                            (1ULL << AIN2_PIN) |
                            (1ULL << BIN1_PIN) |
                            (1ULL << BIN2_PIN) |
                            (1ULL << S0_PIN) |
                            (1ULL << S1_PIN) |
                            (1ULL << S2_PIN) |
                            (1ULL << S3_PIN);
    out_conf.mode = GPIO_MODE_OUTPUT;
    out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&out_conf);

    // Input pins (button) - Zero Initialized!
    gpio_config_t btn_conf = {};
    btn_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    btn_conf.mode = GPIO_MODE_INPUT;
    btn_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    btn_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&btn_conf);
}

static void init_pwm(void)
{
    // Timer config (shared between both channels)
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = PWM_RESOLUTION;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = PWM_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    // Channel 0 — PWMA
    ledc_channel_config_t ch0 = {};
    ch0.gpio_num = PWMA_PIN;
    ch0.speed_mode = LEDC_LOW_SPEED_MODE;
    ch0.channel = LEDC_CHANNEL_0;
    ch0.timer_sel = LEDC_TIMER_0;
    ch0.duty = 0;
    ch0.hpoint = 0;
    ledc_channel_config(&ch0);

    // Channel 1 — PWMB
    ledc_channel_config_t ch1 = {};
    ch1.gpio_num = PWMB_PIN;
    ch1.speed_mode = LEDC_LOW_SPEED_MODE;
    ch1.channel = LEDC_CHANNEL_1;
    ch1.timer_sel = LEDC_TIMER_0;
    ch1.duty = 0;
    ch1.hpoint = 0;
    ledc_channel_config(&ch1);
}

static void init_adc(void)
{
    // Initialize ADC1 in oneshot mode (IDF v5+)
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    // Configure the pot channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12, // 0-3.3V range (DB_11 renamed to DB_12 in IDF v5)
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, POT_CHANNEL, &chan_cfg);
}

// ─────────────────────────────────────────────
// MAIN
// ─────────────────────────────────────────────

extern "C" void app_main(void)
{
    encoder_mutex = xSemaphoreCreateMutex();

    init_gpio(); // Configures pins, installs ISR service, but NO handler_add yet
    init_pwm();
    // init_adc();
    init_pcnt();

    // // Attach encoder ISRs only after all hardware is ready
    // gpio_isr_handler_add(MUX1_SIG, encoder_c1_isr, NULL);
    // gpio_isr_handler_add(MUX2_SIG, encoder_c2_isr, NULL);

    // Clear shift register and select first motor BEFORE tasks start
    shift_out_16(0);
    vTaskDelay(pdMS_TO_TICKS(10));
    select_motor(0);

    printf("Motor array ready. %d motors. Button cycles active motor.\n", TOTAL_MOTORS);

    // Launch tasks last
    xTaskCreate(pid_task, "pid", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 3, NULL);
    // xTaskCreate(adc_task, "adc", 2048, NULL, 3, NULL);
}