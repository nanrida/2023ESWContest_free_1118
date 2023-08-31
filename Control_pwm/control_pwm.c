/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief  PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 *
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

#define FREQ_IN_US 5000L  // 100 microsec

#define Pwm_Pin1   NRF_GPIO_PIN_MAP(0,13)
#define L_Dir_Pin1  NRF_GPIO_PIN_MAP(0,11)
#define L_Dir_Pin2  NRF_GPIO_PIN_MAP(0,12)

#define Pwm_Pin2   NRF_GPIO_PIN_MAP(0,24)
#define R_Dir_Pin1  NRF_GPIO_PIN_MAP(0,14)
#define R_Dir_Pin2  NRF_GPIO_PIN_MAP(0,15)

#define MOTOR_ACTIVE_STATE 0

static volatile bool ready_flag;            // A flag indicating PWM status.
void control_pwm_set();

/*PWM callback function*/
void pwm_ready_callback(uint32_t pwm_id)    
{
    ready_flag = true;
}

void stop()
{
    nrf_gpio_pin_write(L_Dir_Pin1,0);
    nrf_gpio_pin_write(L_Dir_Pin2,0);
    nrf_gpio_pin_write(R_Dir_Pin1,0);
    nrf_gpio_pin_write(R_Dir_Pin2,0);
     while(app_pwm_channel_duty_set(&PWM1,0,1) == NRF_ERROR_BUSY);
     while(app_pwm_channel_duty_set(&PWM1,1,1) == NRF_ERROR_BUSY);
}
void go(int l,int r)
{
     nrf_gpio_pin_write(L_Dir_Pin2,1);
     nrf_gpio_pin_write(R_Dir_Pin2,1);
     nrf_gpio_pin_write(L_Dir_Pin1,0);
     nrf_gpio_pin_write(R_Dir_Pin1,0);
     while(app_pwm_channel_duty_set(&PWM1,0,l) == NRF_ERROR_BUSY); 
     while(app_pwm_channel_duty_set(&PWM1,1,r) == NRF_ERROR_BUSY);
}

void go_right(int l)
{
    int l_value = 15+l;
    int r_value =15;

    /*Maximum value between motor drive and pwm is 99*/
    if(l>=84)
    {
      l_value = 99;
      l -= 84;
      r_value -= l;
      if(r_value <=1)
        r_value =1;
    }
    
     nrf_gpio_pin_write(L_Dir_Pin2,1);
     nrf_gpio_pin_write(R_Dir_Pin2,1);
     nrf_gpio_pin_write(L_Dir_Pin1,0);
     nrf_gpio_pin_write(R_Dir_Pin1,0);
     while(app_pwm_channel_duty_set(&PWM1,1,r_value) == NRF_ERROR_BUSY);
     while(app_pwm_channel_duty_set(&PWM1,0,l_value)== NRF_ERROR_BUSY);     
}
void go_left(int r)
{
    int r_value = 15+r;
    int l_value =15;
    if(r>=84)
    {
      r_value = 99;
      r -= 84;
      l_value -= r;
      if(l_value <=1)
        l_value =1;
    }

     nrf_gpio_pin_write(L_Dir_Pin2,1);
     nrf_gpio_pin_write(R_Dir_Pin2,1);
     nrf_gpio_pin_write(L_Dir_Pin1,0);
     nrf_gpio_pin_write(R_Dir_Pin1,0);
     while(app_pwm_channel_duty_set(&PWM1,1,r_value) == NRF_ERROR_BUSY); 
     while(app_pwm_channel_duty_set(&PWM1,0,l_value) == NRF_ERROR_BUSY);     
}

void control_pwm_set()
{

    nrf_gpio_cfg_output(L_Dir_Pin1);
    nrf_gpio_cfg_output(L_Dir_Pin2);
    nrf_gpio_cfg_output(R_Dir_Pin1);  
    nrf_gpio_cfg_output(R_Dir_Pin2);        
    ret_code_t err_code;
    
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(FREQ_IN_US, Pwm_Pin1, Pwm_Pin2);

    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

    bsp_board_init(BSP_INIT_LEDS);
    stop();


}
 
