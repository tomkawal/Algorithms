/** base of the example:
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * All rights reserved.
 *==============================
 * code by T.Kawala @Sensata Technologies '2017
 * fast sampling of max.4 ADC channels, ESB radio, RTC
 ******************************************************/

// Aug 17: port to SDK14, fixed,compressed,short & frequent Tx data frames to reduce current pulsation
// Sep 14: work on road strike detection, signal data selection @ max frame of 80 compressed points.
//         Gyro removed for clarity
//				 Dead code removed
//         Comments for sharing with code with Aubrey

//------- BOARD definitions, easier than frequent updates in project defines
//------- define one option only!!
//------- include  boards.c below and NOT boards.h file
//------- DO NOT add boards.c to the project .c files

//define BOARD_PCA10040 //Dev.kit
#define TKA_TMS_GYRO //sensor PCB
//#define other board
//removed: 
//CONFIG_GPIO_AS_PINRESET


//#define FCAL_ENABLED


// Definition of RF channels, which are spaced well apart 
// 0..8 - with 12 MHz spacing
// Prefer EVEN numbers 0,2,4,6, which land in WIFI whitespaces
//#define RF_CH  6                     // 4 used by default (TKa). Reason: It is BLE advertising free.
//#define RF_CHANNEL  12 * RF_CH + 2   // RF channel 0..100, originally default was 2 (2402 MHz)

// (on 4th channel)
//#define MIC_ENABLE     //MEMS microphone


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "boards.c"

#include "nrf.h"
#include "nrf_temp.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_clock.h"

#include "nrf_esb.h"
#include "app_error.h"
#include "nrf_esb_error_codes.h"



#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_power.h"
#include "nrf_saadc.h"



#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"
#include "fds.h"
#include "math.h"

#include "mem_manager.h"
//ported to C from class LeastSqFit (LSQ_ )
//LSQ_Allocate2DArray() !!!first, with n = size; h=0;
//LSQ_Free2DArray()     !!!last

#ifdef M_TEST
int n;
long double **h;
//--------------------------------
// remove 0 on the diagonal by adding of equations.
bool LSQ_tozero ( int i )
{  for (int j = i+1; j < n; j++)
    { if (h[j][i] != 0)
       {  for (int k=0;  k <= n; k++ ) h[i][k] =  h[i][k] +  h[j][k] / h[j][i];
          return(true);
       }
    }
  return(false); //equation unresolvable
}//---------------------------------------------------
bool LSQ_hornerscheme()
{ int   idx;
  long double  t;
  for ( idx = 0; idx <= n-1; idx++)
    { if (( h[idx][idx] == 0 ) && ( LSQ_tozero(idx) == false))   return(false);
    // divide the equation so that the current element becomes 1
      if ( h[idx][idx] != 1 )
        {   t = 1 / h[idx][idx];
            h[idx][idx] = 1;
            for (int j = idx + 1; j <= n; j++ )  h[idx][j] = h[idx][j] * t;
        }
      for (int j = 0; j <=  n-1; j++)
        {  if ( (idx != j) && ( h[j][idx] != 0) )
            {  t = h[j][idx];
               for (int k = idx; k <= n; k++)  h[j][k] = h[j][k] - h[idx][k] * t;
            }
        }
    }
  return( true);
} //-----------------------------------------------
void LSQ_Allocate2DArray( void )
{  h = NULL; 
	 h = nrf_calloc(n,sizeof(long double));  //new long double*[n]; //n+1 rows
   for( int i=0; i < n; i++)  //Row by row
    {  h[i] = nrf_calloc(n+1,sizeof(long double));
			   //new long double[n+1];     //n elements of each column
    }
   for (int i=0; i < n; i++)  //n+1 rows
   for (int j=0; j < n+1; j++ )  h[i][j] = 0;
} //----------------------------------------------------
void LSQ_Free2DArray(void)  //free the allocated memory
{  //free each h[i] buffer and then h itself!!
   for( int i = 0 ; i < n ; i++ ) //rows
     nrf_free(h[i]);   // delete[] h[i]; //row by row
     nrf_free(h); //delete[] h;
}
//------------------------------------------------
void LSQ_AddValue( long double* v )
{   for (int i=0; i <  n; i++)
    for (int j=i; j <= n; j++) h[i][j] = h[i][j] + v[i] * v[j];
} //------------------------------------------------
bool LSQ_GetResult( long double* v )
{ bool ret;
    for (int i=1; i <= n-1; i++ )
    for (int j=0; j <= i; j++ )   h[i][j] = h[j][i];
    ret = LSQ_hornerscheme();
    for (int i=0; i <= n-1; i++ ) v[i] = h[i][n];
    return(ret);
}//------------------------------------


long double power(int a, int b)
{
     long double c=a;
	   if (b == 0) return 1;
     for (int n=b; n>1; n--) c*=a;
     if (b < 0) return(1/c); else return c;
}
//TEST function for LSQ:
//prepare test data
//check return: expected coefficients of least squares 
long double test_LSQ(void)
{	int  i,j,z;
  bool success;
	long double ret_val;  
//  long double Ex; //Re-calc.  for verification
  n = 3;  //ax^2+bx+C  equ order makes the size of the array!
  z = 3;  //nr of data points
  //CHECK you have enough points for polynomial!
  //if (n < m)       return; //more points necessary!
	
	//double *y = nrf_calloc(n,sizeof(double)); //	double *y = new double[n];      
  //double *x = nrf_calloc(n,sizeof(double)); // double *x = new double[n];   
  long double *coef = nrf_calloc(n,sizeof(long double)); //long double *coef = new long double[n]; 
  long double *v = nrf_calloc(n,sizeof(long double)); //long double *v = new long double[n+1]; 
 
	static const int x[3]={12,15,18};
	static const int y[3]={32,36,44};
	// for ( i = 0; i < z; i++ )  { x[i] = ;     y[i] = ;     }

  LSQ_Allocate2DArray(); //with n as the size
  for (i=0; i < z; i++)        // all rows ( samples )
     { // v[0] = 1;  v[1] = x[i]; v[2] = x[i]^2;
        //if (n > 2) //more than a*x+b  polynomial
        // { 
			  for (j=0; j<n; j++)  v[j] = power(x[i],j);
        // }  
        v[n] = y[i]; //Y:expected result in last column
        LSQ_AddValue(v);
     }
    success = LSQ_GetResult(coef);
    LSQ_Free2DArray(); 
    
//recalc  on X
 /*   
   for ( i=0; i<n; i++)   // all rows ( samples )
    {  Ex = coef[0] + coef[1] * x[i];
           //more than a*x+b
       if (n > 2)
         { for (j=2; j<m; j++)
           Ex = Ex + coef[j] * IntPower( x[i], j);
         }
    }
*/
	 ret_val = coef[0];
 //nrf_free (y);
 //nrf_free (x);
 nrf_free (coef);
 nrf_free (v);
		 
  if (success)  return ret_val; else return 0;		 
}
#endif	
static int8_t RSSI_Tx = 0;
static int8_t RSSI_Rx = 0;

//------------------------------------------
// DATA PACKET DEFINITIONS
//------------------------------------------
//max payload len is 252
//big compressed packet is more energy and memory efficient
#define Payload_Offset      4  //9      //FIXED length data header in Tx frame
//4B Header:  AN=1/PCH=0, PCL,   BP/DB,   Th,      Reading1 Absolute, Reading 2 Differential.... 
//#define DATA_PROTOCOL       2      //FIXED protocol version which implies defined below settings
//ADC
#define ADC_CHANNELS        3 		  //XYZ of accelerometer
#define MAX_OUT_SAMPLES     80 			//ADC_SAMPLES ('decimated': 1/4 th for OVS =4)
#define OVS								  4			  //choose oversampling factor (4)
uint8_t   OUT_SAMPL;
#define   MAX_ADC_SAMPLES       (MAX_OUT_SAMPLES * OVS) 	        // for max packet size
uint16_t  ADC_SAMPL;
#define   MAX_SAMPLES_IN_BUFFER (ADC_CHANNELS * MAX_ADC_SAMPLES)  // for max packet size
uint16_t  SAMPL_IN_BUFFER;

//In one packet explained:  header[Payload_Offset]   AND per Channel: 2B ABSolute  + 1B differential all other samples     
#define Payload_max_num    Payload_Offset + ADC_CHANNELS * ( 2 +  ( MAX_OUT_SAMPLES - 1 ) )       
uint8_t payload_test[Payload_max_num]; 

uint16_t MAX_PACKET_COUNT;

void ADC_data_init( uint8_t DATA_PROT )
{ if 	( DATA_PROT == 1 ) //continous packets of 5 samples
	 { OUT_SAMPL = 5;
		 MAX_PACKET_COUNT = 2000;
	 }	
  else //DATA_PROTOCOL == 2 Strike detection
   { OUT_SAMPL = 80;
		 MAX_PACKET_COUNT = 125;
	 }
	 ADC_SAMPL = OUT_SAMPL * OVS;
	 SAMPL_IN_BUFFER = ADC_CHANNELS * ADC_SAMPL;
}	



// multiplexing bit in header for sending some data like temperature/RSSI alternatively
static bool multiplex = false;  
static int roll = 0;  
static int last_roll = 0;
//-------------------------------------------
//flash data storage variables and defs
//-------------------------------------------
static volatile uint8_t fds_write_flag = 0;
static volatile uint8_t fds_init_flag  = 0;
//rec_key
#define 	FILE_ID     					0x1100
#define 	REC_KEY 							0x2200
#define   CAL_KEY     	        0x3300
//#define REC_KEY_RADIO					0x3333


uint32_t flash_record_read_back[3]; //[0] - comms  [1] - config [2] - cal_flag
///
static uint32_t write_var[3]; //has to be static (stay in memory for the time of write operation)
static uint32_t flash_record_cal[3]; //XYZ for read and write

static uint8_t RADIO_channel = 4;
static uint8_t startup_RADIO_channel = 0xff;

#define RADIO_CH      250
#define Normal_Z 			251
#define Revers_Z 			252
#define F_calibration 253 
static int Rev=1; //Normal
static uint8_t esb_app_protocol = 1;
static uint8_t startup_esb_app_protocol = 0;
//static uint8_t Z_reversed = Normal_Z; 
//static uint8_t FCalibrate = 0;
static bool FCAL = false;
static int16_t CAL_XYZ[3] = {0,0,0};

uint8_t new_esb_app_cmd = 0;
uint8_t esb_app_RFchannel = 2;

static ret_code_t fds_test_write( uint32_t * value, uint32_t key, uint32_t length );
static ret_code_t fds_test_find_and_delete ( uint32_t key );

void TX_STATUS(uint8_t status);
void turn_down(void);
void turn_down_idle_period(void);

//-------------------------------------------------------
// SIMPLE FILTER definitions and variables
//-------------------------------------------------------
//==== initially CODE below was ported from OLD prototype (SCR) with 8 bit micro
#define CONV_WIDTH_5   4 
#define WINDOW_WIDTH_5 2 * CONV_WIDTH_5     //at least 2*CONV_WIDTH samples window?
#define SAMPLES_5      2 * WINDOW_WIDTH_5 + 1

#define CONV_WIDTH_9   10 
#define WINDOW_WIDTH_9 2 * CONV_WIDTH_9     //at least 2*CONV_WIDTH samples window?
#define SAMPLES_9      2 * WINDOW_WIDTH_9 + 1
//*************************************************************************
// example of original Savitzky-Golay (SG) smoothing filter using symmetrical
// 9 point parabolic convolution (scaled for integer arithmetic)
// with equation: -5*x^2 + 59
//
// symmetrical filter is ZERO_PHASE (does not alter the signal phase)
// Much of the information about the shape of the pulse  
// is stored in PHASE, rather than in magnitude
//
//     coefficients:                            0..CONV_WIDTH + NORM at the end
static const int conv_coeff_5[CONV_WIDTH_5 + 2] = { 64,53,29,11,3, 256 }; //gaussian filter - can be run recursively! 
static const int conv_coeff_9[CONV_WIDTH_9 + 2] = { 256,242,205,155,105,63,34,17,7,3,1, 1920};
                                          //gaussian filter - can be run recursively! 

    //{ 59,54,39,14,-21, 231 }; old Jack's parabolic filter
static int16_t  ADC_buffer[ WINDOW_WIDTH_9][ADC_CHANNELS]; //old record buffer to run with new data  
static int16_t  ADC_result[ WINDOW_WIDTH_9 + MAX_ADC_SAMPLES ][ADC_CHANNELS];  
static int16_t  conv_result[MAX_ADC_SAMPLES][ADC_CHANNELS];

static int16_t  out_data[MAX_OUT_SAMPLES][ADC_CHANNELS];

//====================================================
// FILTER: the result which is sent out is decimated 
// and delayed only on air by window_width and packet length samples
// in that case approx. 1ms 
//====================================================
void get_data_conv_5(void)//and decimate
{ int i,j,k;
  int c;   
	int conv; //calculated k times
    //convolute calculation possible from CONV_WIDTH element from each end of the table
  for ( k = CONV_WIDTH_5; k < ADC_SAMPL+CONV_WIDTH_5; k++ )
	 for ( j = 0; j < ADC_CHANNELS; j++ ) //all  channels
     { //perform convolution calc. on elements +/-CONV_WIDTH from k index + element k once
             conv  =  ADC_result[k][j];//peak_sweep_buf[k]; //0 element
             conv *=  conv_coeff_5[0];
             for ( i = 1; i <=  CONV_WIDTH_5; i++) // conv width elements
              {      // add + side of k element
                      c  =  ADC_result[k+i][j]; 
                      c *=  conv_coeff_5[i];           
								      conv += c;  
                     // add - side of k element
                      c  =  ADC_result[k-i][j]; 
                      c *=  conv_coeff_5[i];   
      								conv += c;  
               }
             conv /= conv_coeff_5[CONV_WIDTH_5 + 1]; //divide by NORM 
             conv_result[k-CONV_WIDTH_5][j] = (int16_t)conv; //save data
    }    // for k,j
}
//======================================
//==== wider, slower filter: 9 
//======================================
void get_data_conv_9(void)//and decimate
{ int i,j,k;
  int c;   
	int conv; //calculated k times
    //convolute calculation possible from CONV_WIDTH element from each end of the table
  for ( k = CONV_WIDTH_9; k < ADC_SAMPL+CONV_WIDTH_9; k++ )
	 for ( j = 0; j < ADC_CHANNELS; j++ ) //all  channels
     { //perform convolution calc. on elements +/-CONV_WIDTH from k index + element k once
             conv  =  ADC_result[k][j];//peak_sweep_buf[k]; //0 element
             conv *=  conv_coeff_9[0];
             for ( i = 1; i <=  CONV_WIDTH_9; i++) // conv width elements
              {      // add + side of k element
                      c  =  ADC_result[k+i][j]; 
                      c *=  conv_coeff_9[i];           
								      conv += c;  
                     // add - side of k element
                      c  =  ADC_result[k-i][j]; 
                      c *=  conv_coeff_9[i];   
      								conv += c;  
               }
             conv /= conv_coeff_9[CONV_WIDTH_9 + 1]; //divide by NORM 
             conv_result[k-CONV_WIDTH_9][j] = (int16_t)conv; //save data
    }    // for k,j
}
//===
/*
void get_data_conv_(void)//and decimate
{ int i,j,k;
  int c;   
	int conv; //calculated k times
    //convolute calculation possible from CONV_WIDTH element from each end of the table
  for ( k = CONV_WIDTH_; k < ADC_SAMPL+CONV_WIDTH_; k++ )
	 for ( j = 0; j < ADC_CHANNELS; j++ ) //all  channels
     { //perform convolution calc. on elements +/-CONV_WIDTH from k index + element k once
             conv  =  ADC_result[k][j];//peak_sweep_buf[k]; //0 element
             conv *=  conv_coeff_[0];
             for ( i = 1; i <=  CONV_WIDTH_; i++) // conv width elements
              {      // add + side of k element
                      c  =  ADC_result[k+i][j]; 
                      c *=  conv_coeff[i];           
								      conv += c;  
                     // add - side of k element
                      c  =  ADC_result[k-i][j]; 
                      c *=  conv_coeff[i];   
      								conv += c;  
               }
             conv /= conv_coeff[CONV_WIDTH + 1]; //divide by NORM 
             conv_result[k-CONV_WIDTH][j] = (int16_t)conv; //save data
    }    // for k,j
} */
//=======================================================
// POF: battery status from the comparator thresholds
//=======================================================
nrf_drv_power_pofwarn_config_t pof_config; 
static bool    fpof = false;
static uint8_t vpof = 8;
static uint8_t batt_meas = 0;

//LED blinks  on dev.kit
//1- packets sent, every 256
//2- NRF_ESB_EVENT_TX_FAILED
//3- 
//4- NRF_ERROR_NO_MEM  - flush, link lost etc & RTC debug

//LED
#ifndef TKA_TMS_GYRO
static uint8_t   LED1_status = 0x01; 
//static uint8_t   LED3_status = 0x01; 
static uint8_t   LED2_status = 0x01; 
static uint8_t   LED4_status = 0x01; 
#endif


//--------------------------------------------
//watchdog channel
//--------------------------------------------
nrf_drv_wdt_channel_id m_channel_id;

void gpio_power()
{	
#ifdef TKA_TMS_GYRO    
    nrf_gpio_cfg_output( VDD_ACC );
		nrf_gpio_pin_write(VDD_ACC, 1 ); //power for the chip
#ifdef MIC_ENABLE		
	  nrf_gpio_cfg_output( VDD_MIC );
		nrf_gpio_pin_write(VDD_MIC, 1 ); //power for the chip
#endif	
#endif	
}
//GPIO
void gpio_init( void )
{
#ifndef TKA_TMS_GYRO
	bsp_board_leds_init();
#else  //all pins disconnected to sleep	
	nrf_gpio_cfg_default(2); //0-1 LF XTAL
	nrf_gpio_cfg_default(3);
	nrf_gpio_cfg_default(4);
	nrf_gpio_cfg_default(5); //6-10 below
	nrf_gpio_cfg_default(11);
	nrf_gpio_cfg_default(12);
	nrf_gpio_cfg_default(13);
	nrf_gpio_cfg_default(14);
	nrf_gpio_cfg_default(15);
	nrf_gpio_cfg_default(16);
	nrf_gpio_cfg_default(17);
	nrf_gpio_cfg_default(18);
	nrf_gpio_cfg_default(19);
	nrf_gpio_cfg_default(20);//21 reset below
	nrf_gpio_cfg_default(22);
	nrf_gpio_cfg_default(23);
	nrf_gpio_cfg_default(24);
	nrf_gpio_cfg_default(25);
	nrf_gpio_cfg_default(26);
	nrf_gpio_cfg_default(27);
	nrf_gpio_cfg_default(28);
	nrf_gpio_cfg_default(29);
	nrf_gpio_cfg_default(30);
	nrf_gpio_cfg_default(31);
#endif
	nrf_gpio_cfg_default(6);
	nrf_gpio_cfg_default(7);
	nrf_gpio_cfg_default(8);
	//nrf_gpio_cfg_default(16);
	nrf_gpio_cfg_default(9);
	nrf_gpio_cfg_default(10);
	nrf_gpio_cfg_default(21);

}






//------------------------
// Temperature
//------------------------
int32_t volatile temp_nrf;
uint8_t	temp_meas=0; 

static uint8_t TX_PWR;    // Transmit power
static uint16_t Packet_tx_counter; // Packet number 
bool fADC_Setup = false;  //  

static uint32_t ADC_sampling_period  = 25;   //25us: 4 times oversampled the 10 Ksps rate
//#define MAX_PACKET_COUNTER 						125    //10ksps by 80 packet len
  //2000   //10 Ksps / 5 samples per packet
  //500	   //10k /20
static uint8_t RTC_SKIP=0;  // for RTC actions timing
static uint8_t RTC_DELAY_COUNT=0;;                                             
   //ESB radio protocol payloads structures					
#define TX_BUF_CNT 16	 
static nrf_esb_payload_t        tx_payload[TX_BUF_CNT];  
static nrf_esb_payload_t        rx_payload;

//TODO Tx_payload MUST be in circular buffer 
//to keep constant ADC logging
//and freely decide what to send and when
//with history in buffer
static int16_t TX_BUF_IN  = 0;
static int16_t TX_BUF_OUT = 0;

void tx_payload_init() {
int n;
	for (n=0;n<TX_BUF_CNT;n++)
	 {tx_payload[n].pipe = 0;
    tx_payload[n].length = Payload_max_num;
		 // protocol version on nibble |Hi 
  	tx_payload[n].data[0]= ( esb_app_protocol << 4) & 0xf0; 
		tx_payload[n].data[1]= n;     // nth packet 
		tx_payload[n].data[2]= TX_PWR;    // 
		tx_payload[n].noack = true;  
	 }
}

uint8_t DEV_ID[6];          // Device unique ID from 2 memory locations
void get_dev_ID(void){
  uint32_t no;
	no = NRF_FICR->DEVICEADDR[0];
	DEV_ID[0] = (no >> 24) & 0xFF;
	DEV_ID[1] = (no >> 16) & 0xFF;
	DEV_ID[2] = (no >>  8) & 0xFF;
	DEV_ID[3] =  no  & 0xFF;
	no = NRF_FICR->DEVICEADDR[1];
	DEV_ID[4] = (no >>  8) & 0xFF;
	DEV_ID[5] =  no  & 0xFF;
}
#define ID_payload_length 10 //4H+6B ID   Length of 'Idle packet'
static nrf_esb_payload_t     dummy_tx_payload; // 'idle packet' structure declaration
void dummy_tx_payload_init() {
	dummy_tx_payload.pipe = 0; 
  dummy_tx_payload.length = ID_payload_length;
	dummy_tx_payload.data[0] = 0;  //protocol - 0 for idle packet
	dummy_tx_payload.data[1] = 0;  //packet counter
	dummy_tx_payload.data[2] = TX_PWR; // radio power or battery
	dummy_tx_payload.data[3] = 0; //temperature
	dummy_tx_payload.data[4] = DEV_ID[0];
	dummy_tx_payload.data[5] = DEV_ID[1];
	dummy_tx_payload.data[6] = DEV_ID[2];
	dummy_tx_payload.data[7] = DEV_ID[3];
	dummy_tx_payload.data[8] = DEV_ID[4];
	dummy_tx_payload.data[9] = DEV_ID[5];
}

void ESB_Tx_error_check(uint32_t err_code)
{	
									switch (err_code)
									{			case 	NRF_SUCCESS:     //noa ack, ignore
															    break;
												case  NRF_ERROR_NO_MEM:	//receiver not in range
#ifndef TKA_TMS_GYRO					
  nrf_gpio_pin_write(LED_4, (LED4_status & 1) ); LED4_status=~LED4_status; //LEDS_ACTIVE_STATE
#endif
											(void) nrf_esb_flush_tx();
											(void) nrf_esb_start_tx();
												break;
											default: break;
									} //err_code
}

//Link Alive is incremented with 'link alive' packets from data logger
// if 0 over defined RTC period, the transmitter goes to power down mode
static int  Link_alive=1;  //no turn off immemdiately on RTC
int  Tx_Packet_counter;
static bool F_is_down = false;



// necessary definitions to make ADC working
//timer driver
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
//two DMA buffers with alternated use
static nrf_saadc_value_t     m_buffer_pool[2][MAX_SAMPLES_IN_BUFFER];
//PPI channel to trigger ADC sampling by timer
static nrf_ppi_channel_t     m_ppi_channel;
//ADC channels parameter definition structures
static nrf_saadc_channel_config_t  channel_0_config;
static nrf_saadc_channel_config_t  channel_1_config;
static nrf_saadc_channel_config_t  channel_2_config;
static nrf_saadc_channel_config_t  channel_3_config;

void wdt_event_handler(void)
{ //NOTE: The max amount of time we can spend in WDT interrupt 
	//is two cycles of 32768[Hz] clock - after that, reset occurs
}
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{//nothing to do here - timer is used with ppi triggering saadc
}
//==========================================
void saadc_sampling_event_init(void)
{   ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

	//in case of slow sampling: uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, ADC_sampling_period );
    nrf_drv_timer_extended_compare(&m_timer,   NRF_TIMER_CC_CHANNEL0,
                                   ticks,  NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false); //no timer interrupt
   // nrf_drv_timer_enable(&m_timer); do it later, starting sampling
	 
	//get these two addresses to declare PPI action:
    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

	// setup ppi channel so that timer compare event is triggering sample task in SAADC 
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
  //declare PPI action
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_disable(void)
{
  ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);
  APP_ERROR_CHECK(err_code);
}


//defines for 'digital comparator', removing the offset
// slow avg on radial (Z) axis corelates with driving speed,
// indicating the thresholds for road strike detection
#define F_AVG  64   //fast averaging
#define S_AVG 512   //slow averaging
static float		avg_slow[ADC_CHANNELS]; // 
static float		avg_fast[ADC_CHANNELS]; // 
//static float		avg_diff[ADC_CHANNELS]; // fast - slow
static int16_t      peak_count[ADC_CHANNELS]; //avg_diff positive counter (debounced)
//static int16_t      radial_offset;   //z channel slow avg for threshold calculation
static float    peak_threshold[ADC_CHANNELS];     //calculated, depends on radial acc
static int16_t      peak_cnt_threshold[ADC_CHANNELS];//calculated, depends on radial acc
static int16_t      packets_to_send; //calculated, depends on radial acc
//static int16_t     
static bool peak_detected = false;     //set from peak detected until packet after transmission ended
static bool peak_condition_detected = false;  //pre condition for peak detetction 
static bool peak_transmission = false; //during peak transmission
static bool peak_transmitted = false; //when finished transmission
static uint8_t peak_countdown=0;     //wait for the peak buffered before transmitting it
static uint8_t packets_countdown=0;
static int16_t wait_countdown = 0;  //calculated what to wait after trasmitting, before next peak can be detected
static int16_t wait_countdown_counter = 0; //actual countdown counter set after peak is transmitted


//====================================
void update_peak_thresholds(void)
{ float A;

	 //peak amplitude minimum, use Rev in compares
	const float  A1 = -0.085; 
  peak_threshold[2] = A1 * avg_slow[2] * Rev;    //radial threshold
	peak_threshold[1]	= peak_threshold[2]/2; //tangential
	
	//peak count minimum: Y always + / from X - 
	const float A2 = 0.0278;
	const float C2 = 60.0;
	A = A2 * avg_slow[2] * Rev + C2;
	peak_cnt_threshold[2]= (int16_t)(A);
	peak_cnt_threshold[1]= peak_cnt_threshold[2];

	//packets to send minimum
  const int16_t packets_minimum	= 3;
	const float A3 = 0.0121;
	const float C3 = 10.4113;
	A = A3 * avg_slow[2] * Rev + C3;
	packets_to_send = (int16_t)(A)*2; //doubled for evaluation
	if (packets_to_send < packets_minimum ) packets_to_send = packets_minimum;
	else 
  if (packets_to_send > (TX_BUF_CNT-2))	packets_to_send = (TX_BUF_CNT-2);
	
  //wait packets before next strike detection allowed
	const float A4 = 150.0; 
	  //observe the sign for sqrt()!
	if ( avg_slow[2] >=0 )   	A = A4 / sqrt(avg_slow[2]); 	
	  else     	A = A4 / sqrt(-avg_slow[2]); 
	
	wait_countdown = (int16_t)(A) - packets_to_send/2;
	if (wait_countdown < (packets_to_send) ) wait_countdown = packets_to_send;
	
}
//========================================================================================
//SAADC callback from interupt after DMA buffer is filled with requested number of samples
//========================================================================================
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{  if (p_event->type == NRF_DRV_SAADC_EVT_DONE) //DMA buffer full
    {		ret_code_t err_code;
	//1st add this buffer back to the queee for double buffering
	//and still have time for processing whilst other buffer is being filled up			
			  err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPL_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
	//If device 'turned down', exit because of dummy packet on RTC, 
	//once every n seconds, is used to keep the link alive			
			if (F_is_down) return; 
  //packet counter always updated
			if (++Packet_tx_counter >= MAX_PACKET_COUNT ) 
			{			Packet_tx_counter=0;			
						if (++roll > 7) roll = 0; //false; else roll = true;
			}	
	int i,j,k;    
	int16_t diff;				 
	int idx;	
	idx=Payload_Offset;		 
			
  int WINDOW_WIDTH;			
		//TODO:	switch(sampling_data_rate)	for window_width!
		WINDOW_WIDTH = WINDOW_WIDTH_5;
			
			//step 1: copy of of old buffer part, memcpy cannot be trusted..like memcpy(ADC_result,ADC_buffer, WINDOW_WIDTH * sizeof(int16_t) );  
    		 for (i=0; i< WINDOW_WIDTH; i++) //8
				    for (j=0; j< ADC_CHANNELS; j++)
				     ADC_result[i][j] = ADC_buffer[i][j];
			//step 2: take the ADC results from sampling buffer 
				 for (i=0; i< ADC_SAMPL; i++) //20
						for (j=0; j< ADC_CHANNELS; j++) 
				 	   ADC_result[WINDOW_WIDTH + i][j] = p_event->data.done.p_buffer[ ADC_CHANNELS * i + j];
			//step 3: filter data low pass, with filter 
			//TODO:	switch(sampling_data_rate)
			get_data_conv_5();
			//step 4: copy last data in buffer for next buffer processigng..ADC_result[WINDOW_WIDTH + ADC_SAMPLES ][4];
				 for (i=0; i < WINDOW_WIDTH; i++) //8
				    for (j=0; j < ADC_CHANNELS; j++)
				     ADC_buffer[i][j] = ADC_result[ADC_SAMPL + i][j];
			
			//step 5: decimate with OVS factor and apply offset 
			//STEP 5A: AVERAGES
  int conv_buf;				
	   for (i=0; i< OUT_SAMPL; i++) // once per sample:
				  { for (j=0; j < ADC_CHANNELS; j++) // ch
            { conv_buf  = 0;//.0;
							for (k=0; k < OVS; k++) //4 is oversampling & decimation ratio
							  conv_buf += conv_result[ OVS*i + k ][j];
							conv_buf /= OVS;
							out_data[i][j] = (int16_t)conv_buf + CAL_XYZ[j]; 
							
							//5A: average    add new   subtract_old
if (esb_app_protocol ==2) 	
    {							
							avg_slow[j] += (conv_buf - avg_slow[j]) / S_AVG; 
						  avg_fast[j] += (conv_buf - avg_fast[j]) / F_AVG; 
		}	
						}
				
if (esb_app_protocol ==2) 	//bother with peak detection and sequence control
    {
						if (!peak_detected)
						 { if (!peak_condition_detected)
							 {								 
      //TANGENTIAL
								  
								if   ( Rev * ( avg_fast[1] - avg_slow[1] ) > peak_threshold[1] ) 
									//|| ( (Rev==-1) && ( ( avg_fast[1] - avg_slow[1] ) < peak_threshold[1] ) ) )
   									peak_count[1]++; 
								else { 
			//positive counter debounce && tricky sequence control to keep for evaluation
			// radial peak is delayed in phase, after tangential
											if ((peak_count[1] > 0) && (peak_count[2] == 0)) peak_count[1]--; 
										}
			//RADIAL		
								if ( Rev * ( avg_fast[2] - avg_slow[2] ) > peak_threshold[2] )   peak_count[2]++; 
								else { 		//with positive counter debounce 
											if (peak_count[2] > 0) peak_count[2]--; 
										} 	
   //verify 17SEP17: more conditions for peak detection							
								if ((peak_count[1] > peak_cnt_threshold[1])  //tangential
									&&(peak_count[2] > peak_cnt_threshold[2]) )//radial
										 peak_condition_detected = true;  

						 } } //if (!peak_detected) { if (!peak_condition_detected)
		} //if (esb_app_protocol ==2)		 
						 
						} //for() once per sample
						
//04 oct 17: CALIBRATION procedure
int32_t calv;						
if (FCAL)
{ //use avg_fast[j] SAVE, clear flag, Reset
 calv = (int32_t)(avg_fast[0]); 
 calv	*= -1;
 flash_record_cal[0] = calv;	

 calv = (int32_t)(avg_fast[1]); 
 calv	*= -1;
 flash_record_cal[1] = calv;	
	
 calv = (int32_t)(avg_fast[2]); 
 calv	*= -1;
// calv += (10 * -Rev) ;	
 flash_record_cal[2] = calv;	
	
//calibration record	
 (void)fds_test_find_and_delete( 1 );
  fds_write_flag=0;	   //this we'll be waiting to set, for finishing the write op.		
  err_code = fds_test_write(flash_record_cal, 1, 3 );
		//if FDS_ERR_NO_SPACE_IN_FLASH call fds_gc() then try write once again														
  if (err_code == FDS_ERR_NO_SPACE_IN_FLASH ) //if all storage space is dirty
		{ (void)fds_gc(); //clean garbage and retry once
			err_code = fds_test_write(flash_record_cal, 1, 3 );
		}
	//wait until write is finished
	if (err_code == NRF_SUCCESS) while (fds_write_flag==0) {};
		
 //clear calibration flag
  write_var[1] = flash_record_read_back[1];	//cfg: copy of read value
 	write_var[0] = flash_record_read_back[0];//comms: copy of read value;		
  write_var[2] = 0;
	
	TX_STATUS(5); //STATUS: CALIBRATION	DONE
  
	(void)fds_test_find_and_delete( 0 );
  fds_write_flag=0;	
	err_code = fds_test_write(write_var, 0, 3 );
	if (err_code == FDS_ERR_NO_SPACE_IN_FLASH ) //if all storage space is dirty
					{ (void)fds_gc(); //clean garbage and retry once
						err_code = fds_test_write(write_var, 0, 3 );
					}
		//wait until write is finished
	if (err_code == NRF_SUCCESS) while (fds_write_flag==0) {};		
													 
	//??? NVIC_SystemReset();	//Reset and start with the new settings
		//or make the part more visible:
	RTC_DELAY_COUNT = 127;		
	if (F_is_down)	turn_down_idle_period(); else turn_down(); 	
}	
						
						
if (esb_app_protocol ==2) 	//if strike detection protocol
    {						
			//once per packet received:		
			//update thresholds, except when peak conditions just detected
      //OR peak condition   
        if (! (peak_condition_detected) ) 	 { 			 update_peak_thresholds();	 }
        else  {  
									if (wait_countdown_counter > 0) 
									 {  wait_countdown_counter--; 
				//if too early for the next peak, reset peak detection;
										  peak_count[0]=0; peak_count[1]=0; peak_count[2]=0;
										  peak_condition_detected = false;
									 }	
									 else 
				//wait no more:
							     { 
										 peak_detected = true;	
										 peak_condition_detected = false;
							       peak_countdown = packets_to_send/2; //to wait for remaining ADC logs of the peak
									 }	 
							} //peak_condition_detected
		 //peak detected: prepare for packets to be sent out		 
           if (peak_detected)					 
				   {  if (!peak_transmission) //packets_to_send /2 +1 from back in time
						 { 
			//peak_countdown: WAIT with declaring transmission after the peak was detected until packets buffered					 
							 if (peak_countdown > 0) peak_countdown--; else
							 { //indicate packets to send from the queue
							 TX_BUF_OUT  =  TX_BUF_IN - 1;
							 TX_BUF_OUT -=  packets_to_send;
							 if (TX_BUF_OUT < 0 ) TX_BUF_OUT += TX_BUF_CNT; //wrap around the capacity e.g. -1+16=15
								peak_transmission = true; 
							 } 
							 last_roll = 0; // prevent sending sync packet anytime soon 
						 }
						 else //peak_transmission
             { 
							 if (peak_transmitted) //reset the state
               { peak_transmitted = false; peak_detected = false; peak_transmission=false;
								 peak_count[0]=0; peak_count[1]=0; peak_count[2]=0;
	//rejected move 21sep 2017 //- move after peak detected??
								 wait_countdown_counter = wait_countdown; 
							 } 
	//peak transmission: don't fill next packets in the buffers!		
  // otherwise it makes no harm  	
							 //anyway: packets are now buffered at once in ESB buffers, commented out:							 
							 //else return; 
					   }
					 }
	}	//	(esb_app_protocol ==2) 	//if strike detection protocol
		
			//step 6:  prepare one ABSolute sample and then other differential
				 for (j=0; j< ADC_CHANNELS; j++) //4ch
							{ tx_payload[TX_BUF_IN].data[idx    ] =  ( out_data[0][j] >> 8) & 0xFF;   //Hi
                tx_payload[TX_BUF_IN].data[idx +1 ] =    out_data[0][j] & 0xFF;         //Lo
                idx+=2;
							}
				 for (i=1; i < OUT_SAMPL; i++) //remaining 4 of 5 (or 79 of 80)
				    for (j=0; j < ADC_CHANNELS; j++) // ch		
						  { diff = out_data[i-1][j] - out_data[i][j]; 
				//TODO: make sure diff is not reaching capacity of signed byte (in GUI logs?)								
							  tx_payload[TX_BUF_IN].data[idx] =  diff & 0xFF;
                idx++;
							} 			
		  	tx_payload[TX_BUF_IN].length  = idx; 	//tx_payload.data[6] = idx;
							
			//step 7: indicate protocol version & multiplex 	&  packet counter: 0..MAX PACKEt COUNTER-1
 			  tx_payload[TX_BUF_IN].data[0]  = (esb_app_protocol << 4) & 0xf0;

       //(esb_app_protocol ==2) 	// strike detection protocol							
				if (esb_app_protocol > 1) 
				{  	tx_payload[TX_BUF_IN].data[0] |= (roll & 0x07); 
				}	  else
				    tx_payload[TX_BUF_IN].data[0] |= (Packet_tx_counter >> 8) & 0x07; // hi bits of packet counter
      
				if (multiplex)	tx_payload[TX_BUF_IN].data[0] |= 0x08; 	
				
				tx_payload[TX_BUF_IN].data[1] =  Packet_tx_counter & 0xff; //lo byte of packet counter				 
			
				 if (multiplex) /*Tx_RSSI*/
					 { tx_payload[TX_BUF_IN].data[2] = RSSI_Tx; 
						 multiplex = false; 
						 tx_payload[TX_BUF_IN].data[3] = 255; //will become RSSI of Rx device
				   } else 				   
				   { multiplex = true; 
					   tx_payload[TX_BUF_IN].data[2] = (vpof << 4) & 0xf0; 	
				     tx_payload[TX_BUF_IN].data[2] += TX_PWR;	
						 tx_payload[TX_BUF_IN].data[3] = (int8_t)(temp_nrf & 0x000000FF);
           }
				
	if (esb_app_protocol ==2)
				{
	  //ACK required only for packet ZERO and in the middle...
				 if  ((Packet_tx_counter == 0 )  || ( Packet_tx_counter == (MAX_PACKET_COUNT/2) ) )
				 { tx_payload[TX_BUF_IN].noack = false;	
					 last_roll++;
				 }	 
				 else
				 	 tx_payload[TX_BUF_IN].noack = true; //should reduce rx current turning reception on just once/s
			 } 
	else if (esb_app_protocol ==1)
			 {
					if  (Packet_tx_counter == 0 ) 
				  tx_payload[TX_BUF_IN].noack = false;	
					 	else
				 	tx_payload[TX_BUF_IN].noack = true;  
          err_code = 	nrf_esb_write_payload(&tx_payload[ TX_BUF_IN ]);
					ESB_Tx_error_check(err_code);				
						
			 }	 
	  //Send a packet ZERO and in the middle to sync.. half seconds	 				 
	  //only if not a part of packet transmission already	( last_roll is reset in transmission)				
			 	   if ((esb_app_protocol ==2) &&( last_roll >= 2 )) //peak_transmission || peak_detected ) )			
					    { last_roll = 0;
			  			  err_code = 	nrf_esb_write_payload(&tx_payload[ TX_BUF_IN ]);
						    ESB_Tx_error_check(err_code);
#ifndef TKA_TMS_GYRO	
										Tx_Packet_counter++;		//for blinking LED							
#endif								
					    }	
		       	
					
			 // nrf_drv_wdt_channel_feed(m_channel_id);    
	   
				//when to update TX_BUF_IN?
				//may be better before than after, because of pilot frame?
					//update the buffer pointer:
        if (++TX_BUF_IN >= TX_BUF_CNT ) TX_BUF_IN = 0;
					 

    //send packets after strike completion					 
  	if ((esb_app_protocol ==2) && (peak_transmission) )
    { 
			packets_countdown = packets_to_send;
 			 do 
					{err_code =  nrf_esb_write_payload(&tx_payload[TX_BUF_OUT]); 
					 if (++TX_BUF_OUT >= TX_BUF_CNT ) TX_BUF_OUT = 0; 
           //transmit and countdown the frames buffered
					//finally set flag peak_transmitted
					if (--packets_countdown == 0)	 peak_transmitted=true;		
					}	
			  while ((err_code == NRF_SUCCESS)  && (peak_transmitted==false));	
	
#ifndef TKA_TMS_GYRO	
					Tx_Packet_counter++;  //for blinking LED
#endif				
		 }		
#ifndef TKA_TMS_GYRO	
		 else if (esb_app_protocol == 1)Tx_Packet_counter++;  //for blinking LED
#endif	
    } //event
}//=============================
/*
void saadc_init_LP(void) //low power setting before switch off?
{	   ret_code_t err_code;
	  channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_0_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	  channel_0_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
#if	( ADC_CHANNELS == 1 )	 //Z only?
	  channel_0_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN2);  
#else
    channel_0_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0);  
#endif
    channel_0_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);	

		//set configuration for saadc channel 1
    channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
    channel_1_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN1);
    channel_1_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);

		//set configuration for saadc channel 2
    channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
    channel_2_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN2);
    channel_2_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);
    
    		//set configuration for saadc channel 3
    channel_3_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_3_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    channel_3_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
		channel_3_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN3); 
		channel_3_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);
 
    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
	if ( ADC_CHANNELS > 1 )
	{ err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
	}
  if ( ADC_CHANNELS > 2 )	
  { err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
	}
  if ( ADC_CHANNELS > 3 )  
	{	err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);
  }
}
*/
//=====================================
void saadc_init(void)
{    ret_code_t err_code;
	  channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_0_config.resistor_n = NRF_SAADC_RESISTOR_VDD1_2;
    channel_0_config.gain       = NRF_SAADC_GAIN1_2;
    channel_0_config.reference  = NRF_SAADC_REFERENCE_VDD4;
	  channel_0_config.acq_time   = NRF_SAADC_ACQTIME_3US;
	  channel_0_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
#if	( ADC_CHANNELS == 1 )	 //Z only?
	  channel_0_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN2);  //0);
#else
    channel_0_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0);  
#endif
    channel_0_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);	

		//set configuration for saadc channel 1
    channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_VDD1_2;
    channel_1_config.gain       = NRF_SAADC_GAIN1_2;
    channel_1_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_1_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_1_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
    channel_1_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN1);
    channel_1_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);

		//set configuration for saadc channel 2
    channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_VDD1_2;
    channel_2_config.gain       = NRF_SAADC_GAIN1_2;
    channel_2_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_2_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_2_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
    channel_2_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN2);
    channel_2_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);
    
    		//set configuration for saadc channel 3
    channel_3_config.resistor_p = NRF_SAADC_RESISTOR_VDD1_2;
    channel_3_config.resistor_n = NRF_SAADC_RESISTOR_VDD1_2;
    channel_3_config.gain       = NRF_SAADC_GAIN1_2;
    channel_3_config.reference  = NRF_SAADC_REFERENCE_VDD4;
    channel_3_config.acq_time   = NRF_SAADC_ACQTIME_3US;
    channel_3_config.mode       = NRF_SAADC_MODE_DIFFERENTIAL;
		channel_3_config.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN3); //NRF_SAADC_INPUT_VDD; ///
		channel_3_config.pin_n      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN7);
 
#ifndef TKA_TMS_GYRO
   channel_0_config.resistor_p = NRF_SAADC_RESISTOR_VDD1_2;
   channel_1_config.resistor_p = NRF_SAADC_RESISTOR_VDD1_2;
	 channel_2_config.resistor_p = NRF_SAADC_RESISTOR_VDD1_2;
#endif	



    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);
	if ( ADC_CHANNELS > 1 )
	{ err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
    APP_ERROR_CHECK(err_code);
	}
  if ( ADC_CHANNELS > 2 )	
  { err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);
    APP_ERROR_CHECK(err_code);
	}
  if ( ADC_CHANNELS > 3 )  
	{	err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);
    APP_ERROR_CHECK(err_code);
  }
 //two buffers are used to allow continuity of sampling and processing
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPL_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPL_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}
//===================ESB 
void dec_power(void){
    //case switch 0,4,8,12,16
	switch (TX_PWR) {
		case 0:  if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG4DBM) == NRF_SUCCESS)  TX_PWR = 4;
		 break;	
		case 4:  if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG8DBM) == NRF_SUCCESS)  TX_PWR = 8;
			break;	
		case 8:  if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG12DBM) == NRF_SUCCESS)  TX_PWR = 12;
			break;	
		case 12: if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG16DBM) == NRF_SUCCESS)  TX_PWR = 15;
			break;	
		case 15:	break;			
			default: break;	
	}	
}///------------------------
void inc_power(void){
	switch (TX_PWR) {
		case 15: if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG12DBM) == NRF_SUCCESS)  TX_PWR = 12;
			break;
		case 12: if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG8DBM) == NRF_SUCCESS)  TX_PWR = 8;
			break;
		case 8:  if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_NEG4DBM) == NRF_SUCCESS)  TX_PWR = 4;
		  break;
		case 4:  if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_0DBM) == NRF_SUCCESS)  TX_PWR = 0;
		  break;
		case 0: break;
		default: break;
	}	 
}//==========================================
void turn_up_adc(void)
{ saadc_init();
  saadc_sampling_event_init();
  saadc_sampling_event_enable();
	nrf_drv_timer_enable(&m_timer);
}	
//------------------
void turn_up(void)
{ 
#ifdef TKA_TMS_GYRO    
    nrf_gpio_cfg_output( VDD_ACC );
		nrf_gpio_pin_write(VDD_ACC, 1 ); //power for the chip
#if ( defined(MIC_ENABLE) || defined(GYRO_PRESENT) )
	  nrf_gpio_cfg_output( VDD_MIC );
		nrf_gpio_pin_write(VDD_MIC, 1 ); //power for the chip
#endif	
#endif	
	turn_up_adc();
}
////====SAVE POWER ===============
void turn_down_peri(void) {
nrf_drv_timer_disable(&m_timer);	
saadc_sampling_event_disable();
nrf_drv_ppi_uninit();	
//saadc_init_LP();	//remove dividers
nrf_drv_saadc_uninit(); 
NVIC_ClearPendingIRQ(SAADC_IRQn);
	while (nrf_esb_is_idle()==false) { }; //any radio transaction to be finished
NVIC_ClearPendingIRQ(RADIO_IRQn);	
 // nrf_drv_wdt_channel_feed(m_channel_id);	
}
//----------------------
void turn_down(void)
{
	//trying to remove any pull_up/pull_down 
	channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	channel_0_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	channel_2_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	nrf_saadc_channel_init(0, &channel_0_config);
	nrf_saadc_channel_init(1, &channel_1_config);
	nrf_saadc_channel_init(2, &channel_2_config);
	
	
turn_down_peri();		
#ifdef TKA_TMS_GYRO    
		nrf_gpio_pin_write(VDD_ACC, 0 ); //power off the acc. chip 
	  nrf_gpio_cfg_default(VDD_ACC);
		nrf_gpio_pin_write(VDD_MIC, 0 ); //power off the acc. chip 
	  nrf_gpio_cfg_default(VDD_MIC);
#endif	
	//try to leave the tx power at 0 dBm	
if (nrf_esb_set_tx_power(NRF_ESB_TX_POWER_0DBM) == NRF_SUCCESS)  TX_PWR = 0;	
F_is_down = true;	 
gpio_init();	
nrf_drv_clock_hfclk_release();
NRF_CLOCK->TASKS_HFCLKSTOP = 1; //stop fast clock
NRF_POWER->DCDCEN = 0;  
}
//===goto sleep again after IDLE STATE Tx 'alive frame' =====
void turn_down_idle_period(void)
{ while (nrf_esb_is_idle()==false); //any radio transaction to be finished
  NVIC_ClearPendingIRQ(RADIO_IRQn);	
 	NVIC_DisableIRQ(RADIO_IRQn);
	nrf_drv_clock_hfclk_release();
	NRF_CLOCK->TASKS_HFCLKSTOP = 1; //stop fast clock
	NRF_POWER->DCDCEN = 0; 
	NRF_CLOCK->TASKS_HFCLKSTOP = 1; //stop fast clock
}	
//ESB===============
//================================================================
//STATUS FRAME, 0: when power is down or >0: reply to command
//contains device ID, status (responses to remote commands etc.)
//================================================================
//22Sep17; reused packet counter as STATUS
void TX_STATUS(uint8_t status)
{	ret_code_t err_code;
	static uint8_t roll_1;
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	NRF_POWER->DCDCEN = 1;   
	NVIC_EnableIRQ(RADIO_IRQn);
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0){}; //no a must: a lot to do, before on air?!
	if (++roll_1 > 7) roll_1 = 0;	
	//	if (++Packet_tx_counter >= MAX_PACKET_COUNTER ) Packet_tx_counter=0;		STATUS instead! 
		dummy_tx_payload.data[0]  = 0; //(DATA_PROTOCOL << 4) & 0xf0;
	  dummy_tx_payload.data[0] |=  roll_1 & 0x07; //(Packet_tx_counter >> 8) & 0x07; 	//3LSBit// hi byte of packet counter
		if (multiplex) dummy_tx_payload.data[0] |= 0x08; 	
		
  	dummy_tx_payload.data[1] =  status; //Packet_tx_counter & 0xff; //lo byte of packet counter				
	
	  if (multiplex) {/*Tx_RSSI*/ dummy_tx_payload.data[2] = RSSI_Tx; multiplex = false; } 
		else 	{ multiplex = true;
					  dummy_tx_payload.data[2] = (vpof << 4) & 0xf0; 	
						dummy_tx_payload.data[2] += TX_PWR;	
					}
		dummy_tx_payload.data[3] = (int8_t)(temp_nrf & 0x000000FF);
		dummy_tx_payload.noack = false; //possible turn on command in return packet
		err_code=nrf_esb_write_payload(&dummy_tx_payload);	 
		ESB_Tx_error_check(err_code);
}

void static nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
 uint32_t radio_set;
//#ifndef TKA_TMS_GYRO
	ret_code_t err_code;  //for fds code
//#endif	
	//In this version:
	//IGNORED SUCCESS/FAILED, because of selective ack used?
    switch (p_event->evt_id)   
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            break;
        case NRF_ESB_EVENT_TX_FAILED:  //no Rx Ack (reports previous frame!)
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:	
				    while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
             {  
							  RSSI_Tx = rx_payload.rssi; // to send in data frame header
//==============Link alive							
							 if ( ( rx_payload.length >= 8) && (rx_payload.data[6] == 0xFE)  )				
									{ 	Link_alive++; //only on correct ID????
	                  //To reduce power, depend on remote RSSI 
	                  //keep 80 as a good value? with margin for degradation...										
										  RSSI_Rx = rx_payload.data[7]; 
										if (!F_is_down) {
										  if ( RSSI_Rx > 75 ) inc_power(); else 
										  if ( RSSI_Rx < 65 ) dec_power();
										}
#ifndef TKA_TMS_GYRO
							nrf_gpio_pin_write(LED_2,LED2_status & 1); LED2_status = ~LED2_status;								
#endif	
          				  break;
									} 
//======turn off all sensors									
									else if ( ( rx_payload.length >= 8) && (rx_payload.data[6] == 0xFF) )
						      { if (F_is_down)	turn_down_idle_period(); else turn_down(); 
										RTC_DELAY_COUNT = 0;
										break; 
						      } 	
//==calibration,settings,turn on	or dedicated turn off								
									else if   ( rx_payload.length >= 14)    //Apply_new_config               		
									{
								int i;
									if ( (rx_payload.data[0] == 'S' )	&& (rx_payload.data[1] == 'E' )	&& (rx_payload.data[2] == 'T' )
										&& (rx_payload.data[3] == 'U' )	&& (rx_payload.data[4] == 'P' )	&& (rx_payload.data[5] == 0xA5 ) ) 							
											{ fADC_Setup = true; 			// accept SETUP only if there's sensor's own ID in the request.
												for (i=0; i< 6; i++)   {  if	(rx_payload.data[8+i] != DEV_ID[i])    fADC_Setup = false;     	}  
											} 
									if (fADC_Setup) 
											{ 
												//TODO 3 OCt 2017: consider STOP operation before any complex operation?
												//read on startup and perform calibration
												//complete protocol spec with the status
												//check GUI to decode as well
												TX_STATUS(1); //command accepted 												
#ifndef TKA_TMS_GYRO									
							nrf_gpio_pin_write(LED_3, LEDS_ACTIVE_STATE);
#endif	
												uint8_t setup_command =  rx_payload.data[7]; //AN field
												bool cmd_valid = false;
												bool No_reset = false;
												switch (setup_command)
													{ case 0: //OFF
																		RTC_DELAY_COUNT = 127;		
												            if (F_is_down)	turn_down_idle_period(); else turn_down(); 
														        cmd_valid = true;
                                    break; 
													case 1: cmd_valid = true; break;
													case 2:	cmd_valid = true; break;
														
													case RADIO_CH:	
														//No_reset = true;
														cmd_valid = true; break;
													case Normal_Z: //251 config Normal Z, no restart required 
														No_reset = true;
														cmd_valid = true; break;
													case Revers_Z:	//252 Reversed Z, no restart required		
														No_reset = true;	
														cmd_valid = true; break;
													case F_calibration: //253 calibration, restart required
														cmd_valid = true; break;     
													default: //not supported
                            cmd_valid = false; break;														 
												  }
                         if (setup_command > 0) //if not OFF
												 { if (cmd_valid) 
													 {  new_esb_app_cmd = setup_command; 
														  //command/protocol changed
												      //if ( esb_app_protocol != new_esb_app_protocol ) 
															// {
																 if ( new_esb_app_cmd < 16) 
																 {  
																	  RTC_DELAY_COUNT = 0;
																	if (new_esb_app_cmd != startup_esb_app_protocol)
																	 { write_var[0] = new_esb_app_cmd; // comms
																	   write_var[0] |= (flash_record_read_back[0] & 0xfffffff0);
																		 //(uint32_t)(RADIO_channel) << 4;
 	                                   write_var[1] = flash_record_read_back[1];   //config: copy of read value;
																	   write_var[2] = 0; // no calibration etc on startup;	
																	   TX_STATUS(2); //mode change 
																   } 
																	else
																	NVIC_SystemReset();
																 }	
															   else 
																 { if (No_reset) //changed config option	other than comms 
																   { 
																		 
																		 write_var[1] = new_esb_app_cmd;		//at the moment only reversed _Z
 	                                   write_var[0] = flash_record_read_back[0];//comms: copy of read value;		
                                     write_var[2] = 0; // no calibration etc on startup;																			 
																	   TX_STATUS(3); //STATUS: settings change	
																		 //Z_reversed = new_esb_app_protocol;
																		 if ( new_esb_app_cmd == Revers_Z  ) Rev = -1; else Rev = 1;
																	   
																	 } else	 
																	 { 
																		 if ( new_esb_app_cmd == RADIO_CH )
																		 { 
																			  radio_set = rx_payload.data[6];
																			 
                                        if ( startup_RADIO_channel != radio_set )
 																			  {
																			 // RADIO_channel = rx_payload.data[6];
																			  write_var[1] = flash_record_read_back[1];  
																			  write_var[0] = flash_record_read_back[0] & 0xffffff0f;
																			  write_var[0] |= (radio_set << 4);
																			  write_var[2] = 0;
																			  }
																				else NVIC_SystemReset();
																		 }
																		 else
																		 {
																		 
																		 write_var[1] = flash_record_read_back[1];	//cfg: copy of read value
 	                                   write_var[0] = flash_record_read_back[0];//comms: copy of read value;		
                                     write_var[2] = new_esb_app_cmd; // perform calibration on startup;																			 
																	   TX_STATUS(4); //STATUS: CALIBRATION	
																		 }
																	 }	 
																 }	 
									              (void)fds_test_find_and_delete( 0 );
          										   	//this we'll be waiting to set, for finishing the write op.		
												        fds_write_flag=0;	
									              err_code = fds_test_write(write_var, 0, 3 );
													
																//if FDS_ERR_NO_SPACE_IN_FLASH call fds_gc() then try write once again														
																if (err_code == FDS_ERR_NO_SPACE_IN_FLASH ) //if all storage space is dirty
																	{ (void)fds_gc(); //clean garbage and retry once
																		err_code = fds_test_write(write_var, 0, 3 );
																	}
																//wait until write is finished
																if (err_code == NRF_SUCCESS) while (fds_write_flag==0) {};
												      // }													
												  if (!No_reset) NVIC_SystemReset();	//Reset and start with the new settings
												}	else TX_STATUS(254); //command not supported
												//TODO 26 sep 17:
//switchable filters for strike detection?												
											 } //command > 0	
								    }	
										else //command rejected: wrong ID
										{		TX_STATUS(255); 
												// SKIP is to delay sending 'sensor alive' frames after receiving wrong request 
												// (addressed to other sensor) 	// TODO: consider random SKIP value?
												if (F_is_down) { turn_down_idle_period(); RTC_SKIP = 2; } 
												break; 
										}
									} // rx_payload.length >= 14)
									}	//rx_payload of case:	NRF_ESB_EVENT_RX_RECEIVED			
						break;
				default: break;					
    }
	  if (F_is_down)
			{	 //prevent back to sleep if data packet may tell to wake-up to sample
	     if (  (p_event->evt_id != NRF_ESB_EVENT_RX_RECEIVED ))	
	      {
			   turn_down_idle_period();   
        }	
	    } 
}
//=========================================================
uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
 //TODO consider change of delay with smaller packet size   
		nrf_esb_config.retransmit_delay         = 1000;
    nrf_esb_config.payload_length           = Payload_max_num; 
 //TODO consider retransmit with low sampling rate   
		nrf_esb_config.retransmit_count         = 0;  //1;

    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = true;
    
    TX_PWR = 0; //the reported transmit power 0..-dBm
    
    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}





/**< Declaring an instance of nrf_drv_rtc for RTC0. */
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); 

static uint8_t RTC_CNT;

//================================================================
/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{ //    if (int_type == NRF_DRV_RTC_INT_COMPARE0)

	if (int_type == NRF_DRV_RTC_INT_TICK)  //8 times a second (longest)
    {	
#ifndef TKA_TMS_GYRO						
		       if  (Tx_Packet_counter > 0)  //blinks faster if packets are sent and slower in sleep
				    { Tx_Packet_counter = 0;
             	nrf_gpio_pin_write(LED_1, (LED1_status & 1) ); LED1_status=~LED1_status; 
						}
#endif
			  if  (++RTC_CNT%32!=0) // 8 per second
				{	
		//fast reporting for 10 minutes from regular turn off
					 if ( (F_is_down) && (RTC_DELAY_COUNT > 0 ) && (RTC_SKIP == 0) )  TX_STATUS(0); 
				}
        else //	if  (++RTC_CNT%32==0) //4 seconds
			   { 
					 if ((F_is_down) )  
						 { 
#ifndef TKA_TMS_GYRO	
										Tx_Packet_counter++;		//for blinking LED							
#endif
							if ( RTC_DELAY_COUNT > 0 ) RTC_DELAY_COUNT--;
							 
							if   (RTC_SKIP == 0) TX_STATUS(0); //on powered down & not on wait
							else if ( RTC_SKIP > 0 ) RTC_SKIP--;
						 } 
							else  //powered up & sampling:  try to decrease power, service of turn_down timeout
						 {             //going to sleep after 256/8 seconds if not link alive
							///			      if ( RSSI_Rx < 72 )  flag_dec_power = true; // dec_power();					
							if (RTC_CNT==0) //256
					     { if (Link_alive == 0) 
								  {  packets_countdown =0; //clear counter of packets remaining to send
										 turn_down();
									}
									else Link_alive=0; 
					     }		
						 }
					} //RTC_CNT%32==0 : 4 second tasks
					//HSLCK operation	 
					//     if (!F_is_down)   { if ( RSSI_Rx > 82 ) flag_inc_power = true; //inc_power();			   } 			
        	//feed the dog 
					//nrf_drv_wdt_channel_feed(m_channel_id);					
					
	//measure temperature			
         switch (temp_meas)
				     { /// Start the temperature measurement(when sleep less freq) & wait
		           case 0: if ((!F_is_down) || (RTC_CNT==5) )
							       { NRF_TEMP->TASKS_START = 1; temp_meas++; }
							    break;
               case 1: if  (NRF_TEMP->EVENTS_DATARDY != 0)
								     { temp_meas=0;     
											 NRF_TEMP->EVENTS_DATARDY = 0;
                      //note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. 
                       temp_nrf = (nrf_temp_read() / 4);
                      // Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. 
                       NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */					  
                     }	
										 break;				 
						 default: break; 
						}
	
		// Battery level  // vpof starts @10, climb up threshold until trig
							/*
	  NRF_POWER_POFTHR_V17 
    NRF_POWER_POFTHR_V18 
    NRF_POWER_POFTHR_V19 
    NRF_POWER_POFTHR_V20 
    NRF_POWER_POFTHR_V22 
    NRF_POWER_POFTHR_V24 
    NRF_POWER_POFTHR_V26 
    NRF_POWER_POFTHR_V28 
	*/
		// do it rarely when powered down	
   if ((!F_is_down) || (RTC_CNT==15) || (batt_meas != 0xff) ) 		 
	  {	
	   if (fpof) /*triggered? then don't climb the ladder*/	
         { if (batt_meas <= 8 ) vpof = batt_meas; //only assign at current end of ladder
					 if ((F_is_down) && (batt_meas != 0xff) ) 
					 { batt_meas = 0xff;
						 pof_config.thr = NRF_POWER_POFTHR_V17; 
				     (void) nrf_drv_power_pof_init(&pof_config);
					   nrf_drv_power_pof_uninit(); //save power 
						 //nrf_drv_common_irq_disable(POWER_CLOCK_IRQn);
					 }	 
					 else 
					 {
					 batt_meas = 0; 
				   pof_config.thr = NRF_POWER_POFTHR_V17; 
				   (void) nrf_drv_power_pof_init(&pof_config);
					 fpof = false; //reset flag only after the lowest step is set
					 }	 
				 } 
			else 	 
			   {  //next step to take if not triggered
					 switch (++batt_meas)
					  { case 1: pof_config.thr = NRF_POWER_POFTHR_V18;  break;
					    case 2: pof_config.thr = NRF_POWER_POFTHR_V19;  break;
							case 3: pof_config.thr = NRF_POWER_POFTHR_V20;  break;
							case 4: pof_config.thr = NRF_POWER_POFTHR_V22;  break;
							case 5: pof_config.thr = NRF_POWER_POFTHR_V24;  break;
							case 6: pof_config.thr = NRF_POWER_POFTHR_V26;  break;
							case 7: pof_config.thr = NRF_POWER_POFTHR_V28;  break;
						     //not triggered so simulate the one more level above 2.8V
						  default:  fpof = true;  break;
 					 } //switch batt-meas
					  (void) nrf_drv_power_pof_init(&pof_config);	
				} //!fpof		
			} //battery-meas			 
   } //interrupt type
}
//=====================================
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
	
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
	  
  //DOC 25.1 Clock source   Page 242  nRF52832_PS_v1.3.pdf
	
	//The RTC module features a 24-bit COUNTER, 
	//a 12-bit (1/X) prescaler, capture/compare registers, and a tick
  //event generator for low power, tickless RTOS implementation.
	
	//The PRESCALER register is read/write when the RTC is stopped. Q: is nrf_drv_rtc_disable ok?
	//The PRESCALER register is read-only once the RTC is STARTed. 
	//Writing to the PRESCALER register when the RTC is started has no effect.
	//Counter increment frequency (not overflow!)
     //fRTC [Hz] = 32768 / (PRESCALER + 1 )  	   //Example: 
	//Desired TICK frequency 8 Hz (125 ms counter period)
  
	    //PRESCALER = round(32768 kHz / 8 Hz) – 1 = 4095  so fRTC = 8 Hz 125 ms counter period
   config.prescaler = 4095;  //1/8 of second: 4095 **** DON't forget: it is 12 bit thing!
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);
	
	//page243:
    //Enable tick event & interrupt as The TICK event is disabled by default.
		//The TICK event enables low power "tick-less" RTOS implementation
		//RTC TICK event rather than the SysTick allows the CPU to be powered down while still keeping scheduling
		//events in the RTC can be individually disabled to prevent PCLK16M & and HFCLK being requested 
		//when those events are triggered. This is managed using the EVTEN register.
		
    nrf_drv_rtc_tick_enable(&rtc,true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

void my_pof_handler(void)
{ fpof=true;	}	


		//wait until the write is finished. while (write_flag==0);

static void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result == FDS_SUCCESS)
            {
               fds_init_flag = 1;
            }
            break;
				case FDS_EVT_WRITE:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							fds_write_flag=1;
						}
						break;
        default:
            break;
    }
}



//in case of: FDS_ERR_NO_SPACE_IN_FLASH call fds_gc() then try write once again

static ret_code_t fds_test_write( uint32_t * value, uint32_t key, uint32_t length )
{		fds_record_t        record;
		fds_record_desc_t   record_desc;
	
		record.file_id              = FILE_ID;
	  switch (key)
			{ case 0:   record.key   		= REC_KEY; break;
			  case 1:   record.key   		= CAL_KEY; break;
				default:  record.key   		= REC_KEY; break;
		  }	
		record.data.p_data          = value;
		record.data.length_words    = length;
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != FDS_SUCCESS) 		{				return ret;		}
		return NRF_SUCCESS;
}

static ret_code_t fds_test_read( uint32_t key )//uint16_t rec_key)
{		fds_flash_record_t  flash_record;
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t err_code, len;
		uint32_t * data;
		uint16_t rec_key;
		
   switch (key) 
		 { case 0:   rec_key = REC_KEY; break;
		   case 1:   rec_key = CAL_KEY; break;
			 default:	 rec_key = REC_KEY; break;
	   }		 
		//find records matching the given key  
		while (fds_record_find(FILE_ID, rec_key, &record_desc, &ftok) == FDS_SUCCESS)
		//if (fds_record_find_by_key(rec_key, &record_desc, &ftok) == FDS_SUCCESS)
		{	err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != FDS_SUCCESS)	{			return err_code;	}
          	// Access the record through the flash_record structure.
				len=flash_record.p_header->length_words; 
				data = (uint32_t *) flash_record.p_data;
			  switch (key) 
				{ case 0:	
				  for (uint8_t i=0;i<len;i++) flash_record_read_back[i] = data[i]; break;
				  case 1:
					for (uint8_t i=0;i<len;i++) flash_record_cal[i] = data[i]; break;	
			    default:
					for (uint8_t i=0;i<len;i++) flash_record_read_back[i] = data[i]; break;
				}	
				// Close the record when done.
			  err_code = fds_record_close(&record_desc);
				if (err_code != FDS_SUCCESS)	{			return err_code;	}
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_test_find_and_delete(uint32_t key)//uint16_t rec_key)
{
		fds_record_desc_t   record_desc;
		fds_find_token_t    ftok;
	  uint16_t rec_key;
		ftok.page=0;
		ftok.p_addr=NULL;
	
	   switch (key) 
		 { case 0:   rec_key = REC_KEY; break;
		   case 1:   rec_key = CAL_KEY; break;
			 default:	 rec_key = REC_KEY; break;
	   }
		// Loop and find records with same ID and rec key and mark them as deleted. 
	while (fds_record_find(FILE_ID, rec_key, &record_desc, &ftok) == FDS_SUCCESS)
	//	while (fds_record_find_by_key(rec_key, &record_desc, &ftok) == FDS_SUCCESS)
		{
			(void)fds_record_delete(&record_desc);
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_test_init (void)
{		ret_code_t ret = fds_register(my_fds_evt_handler);
		if (ret != FDS_SUCCESS) 		{ 			return ret;		}
		ret = fds_init();
		if (ret != FDS_SUCCESS) 		{				return ret;		}
		return NRF_SUCCESS;
}



/**
 * @brief Function for main application entry.
 */
#ifdef M_TEST
static bool memory_OK;
#endif

int main(void)
{  
#ifdef FCAL_ENABLED
	uint8_t FCalibrate = 0;
#endif	
	
	  uint8_t Z_reversed = Normal_Z;
    ret_code_t err_code;  
	  gpio_init();
	  gpio_power();
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	  NRF_POWER->DCDCEN = 1;  
		get_dev_ID(); //This is unique ID, default for Bluetooth, reused here  
	                //some bits masked with BLE to meet BT SIG spec.
	  
#ifdef M_TEST
	//TODO: use memory manager:
	 if (nrf_mem_init() == NRF_SUCCESS) memory_OK = true; else memory_OK=false;
	
	 if (memory_OK) test_LSQ();
#endif	 
	// esb_app_protocol = 1;		
	  lfclk_config(); //32kHz low power timing
    rtc_config();   //with RTC
	  nrf_temp_init(); //temperature sensor
		
			//battery level measurement
			//the thresholds are managed in RTC handler
		pof_config.thr = NRF_POWER_POFTHR_V17;
		pof_config.handler = my_pof_handler;
   (void) nrf_drv_power_pof_init(&pof_config);
	
	
	  //err_code = fds_test_init(); //APP_ERROR_CHECK(err_code);
	  if (fds_test_init()==NRF_SUCCESS)
		 { while (fds_init_flag==0){}; //wait for init complete, make sure I can read
  	   if (fds_test_read( 0 ) == NRF_SUCCESS) //FDS_SUCCESS)
	 	   { //setup protocol and radio variables
				 startup_esb_app_protocol = (uint8_t)(flash_record_read_back[0] & 0x0000000f);
				 startup_RADIO_channel    = (uint8_t)(flash_record_read_back[0] & 0x000000f0) >> 4;	
      				 
				 Z_reversed       = (uint8_t)(flash_record_read_back[1] & 0x000000ff);
				 
#ifdef FCAL_ENABLED				 
				 FCalibrate       = (uint8_t)(flash_record_read_back[2] & 0x000000ff);
#endif
				 
			 } 
			 if ( startup_RADIO_channel >= 5) RADIO_channel = 4; else RADIO_channel = startup_RADIO_channel;
			 
			 if (Z_reversed == Revers_Z  ) Rev = -1; else Rev = 1;
			 
#ifdef FCAL_ENABLED
			 if ( FCalibrate == F_calibration )
			 { FCAL = true;
				 esb_app_protocol = 2; //use max packet size;
				 for ( int i = 0; i < 3; i++ ) CAL_XYZ[i] = 0;
       } 
       else
       {				 
			   if (fds_test_read( 1 ) == NRF_SUCCESS) //1:calibration offsets
	 	     { for ( int i = 0; i < 3; i++ ) CAL_XYZ[i] = (int16_t)(flash_record_cal[i]);
			   } 
				 if ( startup_esb_app_protocol <= 2 ) esb_app_protocol = startup_esb_app_protocol;
			 }
#else
		for ( int i = 0; i < 3; i++ ) CAL_XYZ[i] = 0;	 
		if ( startup_esb_app_protocol <= 2 ) esb_app_protocol = startup_esb_app_protocol;	 
#endif	
			 
		 }	
#ifndef TKA_TMS_GYRO			 
		nrf_gpio_pin_write(LED_4, LEDS_ACTIVE_STATE ); 
#endif		 
	  tx_payload_init();  
	  dummy_tx_payload_init();		
	
	  ADC_data_init( esb_app_protocol );
	
		 
	
 	 //radio protocol		
    err_code = esb_init();     APP_ERROR_CHECK(err_code);
	  while (nrf_esb_is_idle()==false){};
		uint32_t RF_CH =  24 * RADIO_channel + 2;
    nrf_esb_set_rf_channel(RF_CH);	
    //nrf_esb_set_rf_channel(RF_CHANNEL);
		
			
			
		TX_STATUS(Z_reversed); //send ID and config
		
		turn_up_adc();	
#ifdef TKA_TMS_GYRO    
    nrf_gpio_cfg_output( VDD_ACC );
		nrf_gpio_pin_write(VDD_ACC, 1 ); //power for the chip
#if ( defined(MIC_ENABLE) || defined(GYRO_PRESENT) )
	  nrf_gpio_cfg_output( VDD_MIC );
		nrf_gpio_pin_write(VDD_MIC, 1 ); //power for the chip
#endif			
#endif		
			

			
  //TODO Configure WDT. Now switched off
//    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
  //  err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    //APP_ERROR_CHECK(err_code);
 //   err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
   // APP_ERROR_CHECK(err_code);
  //  nrf_drv_wdt_enable();
		//  nrf_drv_wdt_channel_feed(m_channel_id);			
	
	//	 turn_down();  
	
    while (1)  //micro goes power saving in main loop if not serving any call backs
    {
 	     __WFE();   //system decides which mode is chosen
       __SEV();
       __WFE();		 
				
			//ARM says: spinning is wasteful, so the lock function could use WFE to enter standby
      // Nordic says: When waiting for something in a loop it's usually best to use __WFE(). 
         // This is to avoid hitting race conditions if you are waiting for a flag for instance.
         //read: https://devzone.nordicsemi.com/question/1555/how-do-you-put-the-nrf51822-chip-to-sleep/#reply-1589
         //and ALL comments in the thread
			
    }//while
}//main
/** @} */
