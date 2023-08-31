/*! ----------------------------------------------------------------------------
 *  @file    ss_twr_initiator.c
 *  @brief   Single-sided two-way ranging (SS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
 *           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
 *           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
 *           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <math.h>

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

#define width 10*0.45
#define height 12*0.45

#if defined(TEST_SS_TWR_INITIATOR)

/* Example application name */
#define APP_NAME "SS TWR INIT v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 3

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16390 
#define RX_ANT_DLY 16390 

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x11, 0x22, 0xab, 0xcd, 0xE0, 0, 0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0xab, 0xcd, 0x11, 0x22, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status regis0ter state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 400

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

double anc_dis[3];          // Array to contain distance from anchor1,2,3
int anc_num = 0;            // Anchor number
int err_check = 0;          // Error check number
double arrive_range =0.06;  // Range of destination
double pos_x=0,pos_y=0;     // Current RC car location
double prev_x=0,prev_y=0;   // Previous RC car location
double tmp_1,tmp_2,tmp_3;   
double g_x ,g_y;            // Destination coordinate
double vehicle_width = 0.1;
double wheel_radius = 0.05;
int timeMs=0;
int newTm=0;
int how=0;                  // Communication count

/* set goal position. first destination: (1.06m,1.65m), second destination: (2.5m,0.83m)*/
double goal_list[2][2] = {{2,8},{2,4}};

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */

/*Interrupt handle*/
void SysTick_Handler()
{
    timeMs++;
}

uint32_t getTickCount()
{
    return timeMs;
}

/*Angle change*/
double change(double theta)
{
  if(theta<0)
  {
    theta*=(-1);
  }
  if(theta>180)
  {
    theta = 360- theta;
  }
  return theta;
}

/*Calculation angle*/
double cal_angle(double x1,double x2,double y1,double y2)
{
/* 57.2957951 means that transfer radian into dgree */
  double angle = atan2(y2-y1,x2-x1)*57.2957951;   

  if(angle<0)
  {
    angle=angle+360;
  }
  return angle;
}
double degree_to_rad(double degree)
{
  double rad = degree * (3.141592 / 180);

  return rad;

}
double ang_velocity(double rad)
{
  double dif_v = rad * (vehicle_width/wheel_radius);    

  return dif_v;

}
/*Calculation triangulation*/
void Get_Pos(double r1, double r2, double r3)
{
    double r1_squared = r1 * r1;
    double r2_squared = r2 * r2;
    double r3_squared = r3 * r3;

    /*0.1 means Height from floor to tag
      1.1 means Height from floor to anchor*/
    double r1_round = r1_squared - (1.1 - 0.1) * (1.1 - 0.1); 
    double r2_round = r2_squared - (1.1 - 0.1) * (1.1 - 0.1);   
    double r3_round = r3_squared - (1.1 - 0.1) * (1.1 - 0.1);

    pos_x = (r1_round - r2_round + width*width) / (2*width);
    pos_y = (r1_round - r3_round + height*height) / (2*height);
    
}

/*Two way range_single side*/
void Twr_ss(int anc_num)
{
/* Loop forever initiating ranging exchanges. */
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;

    if (status_reg & DWT_INT_RXFCG_BIT_MASK)
    {
        uint16_t frame_len;

        /* Clear good RX frame event in the DW IC status register. */
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_getframelength();
        if (frame_len <= sizeof(rx_buffer))
        {
           dwt_readrxdata(rx_buffer, frame_len, 0);

           /* Check that the frame is the expected response from the companion "SS TWR responder" example.
            * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
           rx_buffer[ALL_MSG_SN_IDX] = 0;
           if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
           {
               uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
               int32_t rtd_init, rtd_resp;
               float clockOffsetRatio;

               /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
               poll_tx_ts = dwt_readtxtimestamplo32();    //tag tx,rx timestamp
               resp_rx_ts = dwt_readrxtimestamplo32();

               /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
               clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

               /* Get timestamps embedded in response message. */
               resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
               resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

              
               /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
               rtd_init = resp_rx_ts - poll_tx_ts;  
               rtd_resp = resp_tx_ts - poll_rx_ts;  

               tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
               distance = tof * SPEED_OF_LIGHT;
    
               anc_dis[anc_num] = distance;
                
               if(distance < 0) // Error
               {
                  err_check++;
               }

               /* Display computed distance on LCD. */
               snprintf(dist_str, sizeof(dist_str), "DIST_%d : %3.2f m",anc_num, distance);
               test_run_info((unsigned char *)dist_str);
           }
        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        printf("anc %d err\n",anc_num);
        err_check++;
    }

    /* Execute a delay between ranging exchanges. */
    Sleep(RNG_DELAY_MS);


}

int ss_twr_initiator(void)
{

    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset and initialize DW chip. */
    reset_DWIC(); /* Target specific drive of RSTn line into DW3000 low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 13 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);


    /*Interrupt start*/
    SysTick_Config(SystemCoreClock / 1000); // 1ms
    NVIC_EnableIRQ(SysTick_IRQn);  

    int size = sizeof(goal_list)/sizeof(goal_list[0]);

    for(int i = 0;i<size;i++)
    {
      g_x = goal_list[i][0]*0.45;
      g_y = goal_list[i][1]*0.45;
      
      while(1)
      {
          newTm = getTickCount();
          
          /* Communicate using an address */
          // anc_1 = 0x11,0x22  anc_2 = 0x33,0x44  anc_3 = 0x55,0x66, tag = 0xab,0xcd
          if(tx_poll_msg[5] == 0x11 && tx_poll_msg[6] == 0x22 && anc_num == 0)
          {              
              Twr_ss(anc_num);       
              
              tmp_1 = anc_dis[anc_num];
              
              tx_poll_msg[5] = 0x33;
              tx_poll_msg[6] = 0x44;
              rx_resp_msg[7] = 0x33;
              rx_resp_msg[8] = 0x44;
              
              anc_num = 1;
          }

          else if(tx_poll_msg[5] == 0x33 && tx_poll_msg[6] == 0x44 && anc_num == 1)
          {
              Twr_ss(anc_num);

              tmp_2 = anc_dis[anc_num];


              tx_poll_msg[5] = 0x55;
              tx_poll_msg[6] = 0x66;
              rx_resp_msg[7] = 0x55;
              rx_resp_msg[8] = 0x66;

              anc_num = 2;          
        
          }

          else if(tx_poll_msg[5] == 0x55 && tx_poll_msg[6] == 0x66 && anc_num == 2)
          {
              Twr_ss(anc_num);

              tmp_3 = anc_dis[anc_num];

              tx_poll_msg[5] = 0x11;
              tx_poll_msg[6] = 0x22;
              rx_resp_msg[7] = 0x11;
              rx_resp_msg[8] = 0x22;

              anc_num = 0;       
          }

          /*If an error occurs, start again from anchor number 1.*/
          if(err_check != 0)
          {
              tx_poll_msg[5] = 0x11;
              tx_poll_msg[6] = 0x22;
              rx_resp_msg[7] = 0x11;
              rx_resp_msg[8] = 0x22;
            
              anc_num = 0;
              err_check = 0;
              stop();         // Wheel stop
              continue;        
        
          }

          /*Communication completed with 3 anchors without errors*/
          if(anc_num == 0)
          {
              int cnt = 0;

              /*If there are more than 2 things with a distance of 0, an error.*/
              for(int i=0;i<3;i++)
              {
                if(anc_dis[i] == 0)
                  cnt++;
              }

              if(cnt>1)
              {
                err_check++;
                continue;
              }

              /*Calculate RC car coordinate*/
              Get_Pos(tmp_1,tmp_2,tmp_3);

              /*Arrive*/
              if(pos_x - g_x < arrive_range && g_x - pos_x < arrive_range && pos_y - g_y < arrive_range && g_y - pos_y < arrive_range)
              {
                stop();
                nrf_delay_ms(1000);
                break;
              }

              /*If no previous coordinate,so get new coordinate*/
              if(how == 0)
              {
                prev_x=pos_x;
                prev_y=pos_y;
                how=1;
                go(20,20);
                timeMs = 0;
                continue;
              }

              /*The cycle of wheel control through pwm. Cycle is 600 ms */
              if(newTm >= 600){
              /*Using Differential kinematic, calculate.*/
                double angle_1,angle_2;
                angle_1 = cal_angle(prev_x,pos_x,prev_y,pos_y)-cal_angle(prev_x,g_x,prev_y,g_y);
                
                angle_2 = cal_angle(g_x,prev_x,g_y,prev_y)-cal_angle(g_x,pos_x,g_y,pos_y);

                double angle = change(angle_1)+change(angle_2);
                
                /*PWM control*/
                double rad = degree_to_rad(angle);
                int dif = ang_velocity(rad)/0.08; //  pwm 1% = 0.08 rad/sec
                printf("%d\n",dif);
                
                if(angle_1 >180)
                  go_left(dif);
                
                else if(angle_1 > 0 && angle_1 < 180)
                  go_right(dif);
                
                else if(angle_1 < 0 && angle_1 > -180)
                  go_left(dif);
                
                else if(angle_1<-180)
                  go_right(dif);
                printf("%g, %g\n",prev_x,prev_y);
                
                printf("%g, %g\n",pos_x,pos_y);
              nrf_delay_ms(1000);
        
                prev_x = pos_x;
                prev_y = pos_y;

                newTm = 0;
                timeMs = 0;

             }// count if
               
          }//if anc num == 0

      }//while
   }//for
}
#endif
