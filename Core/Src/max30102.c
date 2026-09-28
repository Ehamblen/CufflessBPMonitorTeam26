#include "max30102_for_stm32_hal.h"

max30102_t max30102_1;


void init max30102(void) {
    //INIT Sensor
    max30102_init(&max30102_1, &hi2c1);
    //Reset the sensor and clear FIFO pointers:
    max30102_reset(&max30102_1);
    max30102_clear_fifo(&max30102_1);

    //Set up sensor configurations:
    // FIFO configurations
    max30102_set_fifo_config(&max30102_1, max30102_smp_ave_8, 1, 7);
    // LED configurations
    max30102_set_led_pulse_width(&max30102_1, max30102_spo2_16_bit);
    max30102_set_adc_resolution(&max30102_1, max30102_spo2_adc_2048);
    max30102_set_sampling_rate(&max30102_1, max30102_spo2_800);
    max30102_set_led_current_1(&max30102_1, 6.2);
    max30102_set_led_current_2(&max30102_1, 6.2);
    // Enter SpO2 mode
    max30102_set_mode(&max30102_1, max30102_spo2);
    // Enable FIFO_A_FULL interrupt
    max30102_set_a_full(&max30102_1, 1);

}

void init_sensor_gpio(void) {
    // Configure GPIO pin for MAX30102 interrupt
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //GPIO Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE(); 

    //GPIO config
    GPIO_InitStruct.Pin = DRDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

    // Enable and set EXTI line interrupt priority
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}