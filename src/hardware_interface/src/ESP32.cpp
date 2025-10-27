#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define TAG "ESP32_CAN_PWM"

// Motor PWM GPIOs
const int motor_gpio[5] = { 18, 19, 21, 22, 23 };  // these gpio spots will be diff based on pwm motor
uint32_t motor_pwm_duty[5] = { 0 };  // initialize for telemetry

// CAN IDs
#define MOTOR_CMD_BASE_ID 0x100  // 0x100 to 0x104 double check with Vin motor ID's might just be filled in from diff file
#define TELEMETRY_REQ_ID  0x200
#define TELEMETRY_RESP_ID 0x201

// PWM config
#define PWM_FREQ_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT

void init_pwm() 
{
    ledc_timer_config_t timer = 
    {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    for (int i = 0; i < 5; ++i) 
    {
        ledc_channel_config_t channel = 
        {
            .gpio_num = motor_gpio[i],
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = static_cast<ledc_channel_t>(i),
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel);
    }
}

void set_motor_pwm(int motor_id, uint32_t duty) 
{
    if (motor_id < 0 || motor_id >= 5)
    {
        return;
    }
    motor_pwm_duty[motor_id] = duty;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(motor_id), duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, static_cast<ledc_channel_t>(motor_id));
}

void init_can() 
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN initialized");
}

void send_telemetry() 
{
    twai_message_t msg = {};
    msg.identifier = TELEMETRY_RESP_ID;
    msg.data_length_code = 5;
    for (int i = 0; i < 5; ++i) {
        msg.data[i] = motor_pwm_duty[i] >> 2;  // Scale 10-bit to 8-bit
    }
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void can_receive_task(void* arg) 
{
    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) 
        {
			// check if message is motor command
            if ((msg.identifier >= MOTOR_CMD_BASE_ID) && (msg.identifier < MOTOR_CMD_BASE_ID + 5)) 
            {
                int motor_id = msg.identifier - MOTOR_CMD_BASE_ID;
                uint32_t duty = msg.data[0] << 2;  // Scale 8-bit to 10-bit
                set_motor_pwm(motor_id, duty);
                ESP_LOGI(TAG, "Motor %d set to duty %d", motor_id, duty);
            }
            else if (msg.identifier == TELEMETRY_REQ_ID) 
            {
                send_telemetry();
                ESP_LOGI(TAG, "Telemetry sent");
            }
        }
    }
}
// extern C says use c linkage for this function helps with esp-IDF
extern "C" void app_main() 
{
    init_pwm();
    init_can();
    xTaskCreate(can_receive_task, "CAN_RX", 4096, NULL, 10, NULL);
}
