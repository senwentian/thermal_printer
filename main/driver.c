#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_types.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define	STEP_DIR_GPIO_NUM    21
#define STEP_DIR_PIN_SEL     (1ULL<<STEP_DIR_GPIO_NUM)

#define STEP_OUTPUT_GPIO_NUM 18
#define STEP_OUTPUT_PIN_SEL  (1ULL<<STEP_OUTPUT_GPIO_NUM)

#define STEP_FAULT_GPIO_NUM  25
// #define STEP_INPUT_PIN_SEL   (1ULL<<STEP_FAULT_GPIO_NUM)
#define STEP_CAP0_INT_EN     BIT(STEP_FAULT_GPIO_NUM)

#define STEP_MODE0_GPIO_NUM  15
#define STEP_MODE1_GPIO_NUM  16
#define STEP_MODE2_GPIO_NUM  17
#define STEP_MODE_PIN_SEL    ((1ULL<<STEP_MODE0_GPIO_NUM) | (1ULL<<STEP_MODE1_GPIO_NUM) | (1ULL<<STEP_MODE2_GPIO_NUM))

#define STEP_LOW      0
#define STEP_HIGHT    1
#define STEP_POSITIVE false
#define STEP_NEGATIVE true

#define STEP_MCPWM_UNIT   MCPWM_UNIT_0
#define STEP_MCPWM_TIMER  MCPWM_TIMER_0
#define STEP_MCPWM_IO_SIG MCPWM0A
#define STEP_MCPWM_CAP0   MCPWM_CAP_0

uint8_t current_mode_type;

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle cap_queue = NULL;

static const char* DRIVER_TAG = "Step Motor Driver";

typedef enum {
    /*Full step (2-phase excitation) with 71% current*/
    STEP_MODE_TYPE_FULL                   = 0,
    /*1/2 step (1-2 phase excitation)*/
    STEP_MODE_TYPE_SUBDIVISION            = 1,
    /*1/4 step (W1-2 phase excitation)*/
    STEP_MODE_TYPE_FOUR_SUBDIVISION       = 2,
    /*8 microsteps/step*/
    STEP_MODE_TYPE_EIGHT_SUBDIVISION      = 3,
    /*16 microsteps/step*/
    STEP_MODE_TYPE_SIXTEEN_SUBDIVISION    = 4,
    /*32 microsteps/step*/
    STEP_MODE_TYPE_THIRTY_TWO_SUBDIVISION = 5,
    /*the num of step mode type*/
    STEP_MODE_TYPE_NUM                    = 6
} step_mode_t;

#define DRIVER_CHECK(a, str, ret) if (a) {                                              \
        ESP_LOGE(DRIVER_TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                 \
        return (ret);                                                                   \
        }

static void mode_gpio_output(uint32_t mode_2, uint32_t mode_1, uint32_t mode_0)
{
    gpio_set_level(STEP_MODE0_GPIO_NUM, mode_0);
    gpio_set_level(STEP_MODE1_GPIO_NUM, mode_1);
    gpio_set_level(STEP_MODE2_GPIO_NUM, mode_2);
}

void motor_set_direction(bool Dir)
{
    gpio_set_level(STEP_DIR_GPIO_NUM, Dir);
}

int motor_get_fault_sig(void)
{
    return gpio_get_level(STEP_FAULT_GPIO_NUM);
}

/**
 * @brief 
 * 
 * @param velocity : the frequency of Pulse, Stepper motor outputs velocity pulses in 1 second
 */
void motor_set_velocity(uint32_t velocity)
{
    mcpwm_set_frequency(STEP_MCPWM_UNIT, STEP_MCPWM_TIMER, velocity);
}

esp_err_t motor_set_subdivision_mode(step_mode_t step_mode_type)
{
    DRIVER_CHECK((step_mode_type > STEP_MODE_TYPE_NUM), "step mode error", ESP_ERR_INVALID_ARG);
    if (step_mode_type != current_mode_type) {
        switch (step_mode_type) {
            case STEP_MODE_TYPE_FULL:
                mode_gpio_output(STEP_HIGHT, STEP_LOW, STEP_LOW);
                current_mode_type = STEP_MODE_TYPE_FULL;
                break;
            case STEP_MODE_TYPE_SUBDIVISION:
                mode_gpio_output(STEP_LOW, STEP_LOW, STEP_HIGHT);
                current_mode_type = STEP_MODE_TYPE_SUBDIVISION;
                break;
            case STEP_MODE_TYPE_FOUR_SUBDIVISION:
                mode_gpio_output(STEP_LOW, STEP_HIGHT, STEP_LOW);
                current_mode_type = STEP_MODE_TYPE_FOUR_SUBDIVISION;
                break;
            case STEP_MODE_TYPE_EIGHT_SUBDIVISION:
                mode_gpio_output(STEP_LOW, STEP_HIGHT,STEP_HIGHT);
                current_mode_type = STEP_MODE_TYPE_EIGHT_SUBDIVISION;
                break;
            case STEP_MODE_TYPE_SIXTEEN_SUBDIVISION:
                mode_gpio_output(STEP_HIGHT, STEP_LOW, STEP_LOW);
                current_mode_type = STEP_MODE_TYPE_SIXTEEN_SUBDIVISION;
                break;
            case STEP_MODE_TYPE_THIRTY_TWO_SUBDIVISION:
                mode_gpio_output(STEP_HIGHT, STEP_LOW, STEP_HIGHT);
                current_mode_type = STEP_MODE_TYPE_THIRTY_TWO_SUBDIVISION;
                break;
            default:
                break;
        }
    }
    return ESP_OK;
}

esp_err_t motor_gpio_init(void)
{
    esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;

    /**
     * GPIO Init -- control subdivision mode of stepper motor
     */
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = STEP_MODE_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    DRIVER_CHECK(ret, "control subdivision mode gpio config error", ret);

    /**
     * GPIO Init -- Direction control
     * 
     */
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = STEP_DIR_PIN_SEL;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    DRIVER_CHECK(ret, "Direction control gpio config error", ret);
    
    // /**
    //  * GPIO Init -- Fault detection input
    //  */
    // //interrupt of falling edge
    // io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // //set as output mode
    // io_conf.mode = GPIO_MODE_INPUT;
    // //bit mask of the pins that you want to set,e.g.GPIO18/19
    // io_conf.pin_bit_mask = STEP_INPUT_PIN_SEL;
    // //enable pull-up mode
    // io_conf.pull_down_en = 1;
    // //configure GPIO with the given settings
    // ret = gpio_config(&io_conf);
    // DRIVER_CHECK(ret, "Fault detection input gpio config error", ret);

    /**
     * GPIO Init -- mcpwm output control
     */
    mcpwm_gpio_init(STEP_MCPWM_UNIT, STEP_MCPWM_IO_SIG, STEP_OUTPUT_GPIO_NUM);
    mcpwm_gpio_init(STEP_MCPWM_UNIT, STEP_MCPWM_CAP0, STEP_FAULT_GPIO_NUM);
    
    // gpio_pulldown_en(STEP_FAULT_GPIO_NUM);    //Enable pull down on CAP0   signal


    return ret;
}

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

static mcpwm_dev_t STEP_MCPWM;

/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler(void *arg)
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = STEP_MCPWM.int_st.val; //Read interrupt status
    if (mcpwm_intr_status & STEP_CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    STEP_MCPWM.int_clr.val = mcpwm_intr_status;
}

void mcpwm_config(void)
{
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(STEP_MCPWM_UNIT, STEP_MCPWM_TIMER, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_set_duty(STEP_MCPWM_UNIT, STEP_MCPWM_TIMER, MCPWM_OPR_A, 50);
    //Capture configuration
    //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
    //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(STEP_MCPWM_UNIT, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    STEP_MCPWM.int_ena.val = STEP_CAP0_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal

    mcpwm_isr_register(STEP_MCPWM_UNIT, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler
}

#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 1500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

void test(void *arg)
{
    bool dump = false;
    uint32_t io_num;
    uint32_t reverse = 1, count;
    capture evt;
    while (1) {
#define pwm_duty_test 0
#if pwm_duty_test

        if (reverse) {
            for (count = 510; count < 1300; count += 10) {
                mcpwm_set_duty_in_us(STEP_MCPWM_UNIT, STEP_MCPWM_TIMER, MCPWM_OPR_A, count);
                printf("count: %d\r\n", count);
                vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
            }
            reverse = !reverse;
        } else {
            for (count = 1300; count > 510; count -= 10) {
                mcpwm_set_duty_in_us(STEP_MCPWM_UNIT, STEP_MCPWM_TIMER, MCPWM_OPR_A, count);
                printf("count: %d\r\n", count);
                vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
            }
            reverse = !reverse;
        }

#endif
// vTaskDelay(100);
#define Dir_test 1
#if Dir_test

        // motor_set_direction(dump);
        // dump = !dump;
        // // printf("the fault level is: %d\r\n", motor_get_fault_sig());
        // vTaskDelay(1000 / portTICK_RATE_MS);

        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            printf("CAP0 : %d us\n", evt.capture_signal);
        }
#endif
    }
}

// static void IRAM_ATTR gpio_isr_level_handler(void* arg)
// {
//     uint32_t gpio_num = (uint32_t) arg;
//     xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
// }

void app_main(void)
{
    motor_gpio_init();

    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(STEP_FAULT_GPIO_NUM, gpio_isr_level_handler, (void*) STEP_FAULT_GPIO_NUM);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    cap_queue = xQueueCreate(1, sizeof(capture)); //comment if you don't want to use capture module

    mcpwm_config();

    printf("Testing motor.......\n");

    motor_set_subdivision_mode(STEP_MODE_TYPE_FULL);
    printf("%d\r\n", STEP_MODE_TYPE_FULL);

    motor_set_subdivision_mode(STEP_MODE_TYPE_THIRTY_TWO_SUBDIVISION);
    printf("%d\r\n", STEP_MODE_TYPE_THIRTY_TWO_SUBDIVISION);

    // motor_set_subdivision_mode(8);

    xTaskCreate(test, "test", 4096, NULL, 5, NULL);
}

