
/* @file       iLoader_mcbstm32c.c
    @brief      This header file implements the iLoader process for a stm32f107 onto a keil board.
                In here we dont use the STM32 in an attempt to keep it slim.
    @author     marco.accame@iit.it
    @date       04/29/2009
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------
#include "string.h"

#include "shalHAL.h"
#include "hal.h"


#include "stdlib.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

//union uParameter {
//  uint8_t     bByte;   //  8 bit unsigned
//  uint16_t    iInt16;  // 16 bit signed
//  uint32_t    iInt32;  // 32 bit signed
//};
//typedef union uParameter USR_tu_Para;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------


//#define _TEST_CAN_
#define _TEST_BASIC_
//#define _TEST_ETH_
//#define _TEST_EEPROM_
//#define _TEST_SPI_



#define LED         0x100

#define APPLADDR    0x8010000

#ifdef _TEST_EEPROM_
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
#endif

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void Delay(uint32_t dlyTicks);
static void LED_Config(void);
static void LED_On(uint32_t led);
static void LED_Off(uint32_t led);





static void s_test_hal_basic(void);

static void s_test_hal_can(void);

static void s_test_hal_eth(void);

static void s_test_hal_spi(void);

static hal_eth_frame_t * s_getframe(uint32_t size);

static void s_giveframeback(hal_eth_frame_t *fr);

static void s_alert_higherlevel(void);

#ifdef _TEST_EEPROM_
static TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void s_test_eeprom(void);
#endif

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile uint32_t msTicks; 

void s_tick(void); 

//inline uint32_t asasa(uint32_t a)
//{
//    return(a=2);
//}                          



#ifdef _TEST_EEPROM_
/* Private define ------------------------------------------------------------*/
#define EEPROM_WriteAddress1    0x50
#define EEPROM_ReadAddress1     0x50
#define BufferSize1             (countof(Tx1_Buffer)-1)
#define BufferSize2             (countof(Tx2_Buffer)-1)
#define BufferSize3             (countof(Tx3_Buffer)-1)
#define EEPROM_WriteAddress2    (EEPROM_WriteAddress1 + BufferSize1)
#define EEPROM_ReadAddress2     (EEPROM_ReadAddress1 + BufferSize1)
#define EEPROM_WriteAddress3    (EEPROM_WriteAddress2 + BufferSize2)
#define EEPROM_ReadAddress3     (EEPROM_ReadAddress2 + BufferSize2)

/* Private macro -------------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
uint8_t Tx1_Buffer[] = "AAABBBCCC";
uint8_t Tx2_Buffer[] = "ABCABC";
uint8_t Tx3_Buffer[] = "AAAAAAAA";
uint8_t Rx1_Buffer[BufferSize1], Rx2_Buffer[BufferSize2], Rx3_Buffer[BufferSize3];
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED, TransferStatus3 = FAILED;
    
#endif










// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------

//#include "stdbool.h"



int main(void) 
{
    extern const hal_params_cfg_t *hal_params_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;
    uint32_t cnt = 0;
    hal_result_t res = hal_res_OK;
    hal_can_frame_t canframe;


    if(ee_res_NOK_generic == shalhal_isvalid())
    {
        for(;;);
    }

    shalhal_init(1);



    hal_memory_getsize(hal_params_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_initialise(hal_params_cfgMINE, data32aligned);

    hal_sys_systeminit();

    // 1 millisec.
    res = hal_sys_systick_sethandler(s_tick, 1000);

    LED_Config();


    if(res == hal_res_OK)
    {
        res = hal_res_NOK_generic;
    } 

#ifdef _TEST_CAN_
    s_test_hal_can();

#endif




#ifdef 	 _TEST_EEPROM_
	 s_test_eeprom();
#endif



#ifdef _TEST_BASIC_
    s_test_hal_basic();
#endif



#ifdef _TEST_ETH_
    s_test_hal_eth();
#endif

#ifdef _TEST_SPI_
    s_test_hal_spi();
#endif




    Delay(5000);

    for(;;)
    {
        Delay(1000);
        LED_On(0);
        Delay(1000);
        LED_Off(0);
        if(++cnt > 10)
        {
        //    hal_sys_systemreset();
        }
    }



}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------




static void s_test_hal_can(void)
{
    hal_can_frame_t canframe;
    hal_result_t res;
    uint8_t count;

    //it is necessary initilize canframe fileds!!
    //hal_can_ functions don't send can frame not well-formed
     
    canframe.id = 0xE;
    canframe.id_type = hal_can_frameID_std;
    canframe.frame_type = hal_can_frame_data;
    canframe.data[0] = 0xCC;
    canframe.data[1] = 0xCC;
    canframe.data[2] = 0xCC;
    canframe.data[3] = 0xCC;
    canframe.data[4] = 0xCC;
    canframe.data[5] = 0xCC;
    canframe.data[6] = 0xCC;
    canframe.data[7] = 0xCC;

    res = hal_can_init(hal_can_portCAN1, NULL);
    if(hal_res_OK != res)
    {
        return;
    }

     res = hal_can_enable(hal_can_portCAN1);
    if(hal_res_OK != res)
    {
        return;
    }


    canframe.size = 8;
    canframe.data[0] = 1;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_later);

    canframe.size = 8;
    canframe.data[0] = 2;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_now);


    canframe.size = 8;
    canframe.data[0] = 3;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_later);

    canframe.size = 8;
    canframe.data[0] = 4;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_now);

    canframe.size = 8;
    canframe.data[0] = 5;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_later);

    canframe.size = 8;
    canframe.data[0] = 6;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_later);

    canframe.size = 8;
    canframe.data[0] = 7;
    hal_can_put(hal_can_portCAN1, &canframe, hal_send_now);

    while(1)
    {
        for(count = 0; count <15; count ++)
        {
            canframe.data[0] = 0xF0 + count;
            hal_can_put(hal_can_portCAN1, &canframe, hal_send_now);
            canframe.data[0] = count;
            hal_can_put(hal_can_portCAN1, &canframe, hal_send_now);
        }
        Delay(1000);
    
    }
}


static void s_test_hal_basic(void)
{
    hal_result_t res;

    uint32_t data04[9];

    uint8_t data01[8];

    uint32_t word = 0x12345678;
    uint16_t hword = 0x1234;
 

    LED_Config();
    LED_On(0);
    LED_Off(0);

    LED_On(0);

    hal_gpio_getval(hal_gpio_portE, hal_gpio_pin8);

//    hal_display_init();
//
//    hal_display_clear(hal_col_white);
//
//    hal_display_settext(hal_font_24x16, hal_col_lightgrey, hal_col_yellow);
//    hal_display_putstring(2, "  hello yellow");
//
//    hal_display_settext(hal_font_24x16, hal_col_red, hal_col_green);
//    hal_display_putstring(3, "  hello green on red ");




    hal_flash_setlatency(hal_flash_1cycles);


    res = hal_flash_unlock();

    res = hal_flash_erase(0x08020003, 1);
    res = hal_flash_write(0x08020000, 4, &word);
    res = hal_flash_write(0x08020007, 2, &hword);

    res = hal_flash_write(0x08020010, sizeof(data04), data04);

    res = hal_flash_write(0x08020040, sizeof(data01), (uint32_t*)data01);

    res = hal_flash_erase(0x08020003, 4096);
    res = hal_flash_write(0x08020010, sizeof(data04), data04);
    res = hal_flash_write(0x08020030, 8, data04);
    res = hal_flash_write(0x08020080, 2, &data04[0]);
    res = hal_flash_write(0x08020090, 2, &hword);
    res = hal_flash_write(0x08020000+4096, sizeof(data04), data04);
    res = hal_flash_erase(0x08020003, 4096);
}


static void s_test_hal_eth(void)
{

    static hal_eth_onframereception_t frx;
    static uint8_t mac[6] = {0, 1, 2, 3, 4, 5};
    static hal_eth_frame_t *pframe;
    static uint32_t datafr[256] = {255};
    pframe = (hal_eth_frame_t*) &datafr[0];

    frx.frame_new                   = s_getframe;
    frx.frame_movetohigherlayer     = s_giveframeback;
    frx.frame_alerthigherlayer      = s_alert_higherlevel;

    pframe->length = 256;
    pframe->index  = 0;
//    frame.data   = datafr;


    hal_eth_init(&frx, mac);

    hal_eth_enable();
    
    hal_eth_sendframe(pframe);
    hal_eth_enable();    
}



static void Delay(uint32_t dlyTicks) 
{
    volatile uint32_t curTicks;

    curTicks = msTicks;
    while((msTicks - curTicks) < dlyTicks);
}






static void callbkspi3(void)
{
    static uint32_t mio_risultato=0;
    
    hal_spi_encoder_get_value( hal_spi_interfSPI3, hal_spiEncoder_1, &mio_risultato );
}



static void s_test_hal_spi()
{
    static uint32_t res;
    uint32_t i;
    hal_spi_init(hal_spi_interfSPI3, callbkspi3);
    
    
    hal_spi_encoder_read_block(hal_spi_interfSPI3, hal_spiEncoder_1, &res);
    hal_spi_encoder_read_block(hal_spi_interfSPI3, hal_spiEncoder_2, &res);
    
    hal_spi_encoder_read_start(hal_spi_interfSPI3, hal_spiEncoder_1);
    while(1)
    {;}

//while(1)
//{
// i = 500;
// hal_spi_encoder_read_start(hal_spi_interfSPI3, hal_spiEncoder_3);
// while(i>0)
// {i--;}
// res = 0;
// res = hal_spi_encoder_read_block(hal_spi_interfSPI3, hal_spiEncoder_1);
//}

}


static void LED_Config(void) 
{
    // assume port E. the LED is led 8
    hal_gpio_init(hal_gpio_portE, hal_gpio_pin8, hal_gpio_dirOUT, hal_gpio_speed_50MHz);
}


static void LED_On(uint32_t led) 
{
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8, hal_gpio_valHIGH); 
}


static void LED_Off(uint32_t led) 
{
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8, hal_gpio_valLOW);  
}


static void s_tick(void)
{
    msTicks++;
}


static hal_eth_frame_t * s_getframe(uint32_t size)
{
    static hal_eth_frame_t *pframe;
    static uint32_t dataframe[256] = {0};

    pframe = (hal_eth_frame_t*)&dataframe[0];
    pframe->length = size;
    pframe->index = 0;
    return(pframe);
}

static void s_giveframeback(hal_eth_frame_t *fr)
{
    static hal_eth_frame_t *fraptr = NULL;
    fraptr = fr;

    if(NULL == fraptr)
    {
        fraptr = fraptr;
    }
}

static void s_alert_higherlevel(void)
{
    static uint32_t alerted = 0;

    alerted ++;
    if(100 == alerted)
    {
        alerted = 99;
        alerted ++;
    }
}



#ifdef _TEST_EEPROM_
/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *         FAILED: pBuffer1 differs from pBuffer2
  */
static TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    
    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;  
}



static void s_test_eeprom(void)
{
   /* Initialize the I2C EEPROM driver ----------------------------------------*/
  hal_eeprom_init();  

  /* First write in the memory followed by a read of the written data --------*/
  /* Write on I2C EEPROM from EEPROM_WriteAddress1 */
  hal_eeprom_WriteBuffer(Tx1_Buffer, EEPROM_WriteAddress1, BufferSize1); 

  /* Read from I2C EEPROM from EEPROM_ReadAddress1 */
  hal_eeprom_ReadBuffer(Rx1_Buffer, EEPROM_ReadAddress1, BufferSize1); 

  /* Check if the data written to the memory is read correctly */
  TransferStatus1 = Buffercmp(Tx1_Buffer, Rx1_Buffer, BufferSize1);
  /* TransferStatus1 = PASSED, if the transmitted and received data 
     to/from the EEPROM are the same */
  /* TransferStatus1 = FAILED, if the transmitted and received data 
     to/from the EEPROM are different */

  /* Wait for EEPROM standby state */
  hal_eeprom_WaitEepromStandbyState();

  /* Second write in the memory followed by a read of the written data -------*/
  /* Write on I2C EEPROM from EEPROM_WriteAddress2 */
  hal_eeprom_WriteBuffer(Tx2_Buffer, EEPROM_WriteAddress2, BufferSize2); 

  hal_eeprom_WriteBuffer(Tx3_Buffer, EEPROM_WriteAddress3, BufferSize3); 

  /* Read from I2C EEPROM from EEPROM_ReadAddress2 */
  hal_eeprom_ReadBuffer(Rx2_Buffer, EEPROM_ReadAddress2, BufferSize2);

  /* Check if the data written to the memory is read correctly */
  TransferStatus2 = Buffercmp(Tx2_Buffer, Rx2_Buffer, BufferSize2);
  /* TransferStatus2 = PASSED, if the transmitted and received data 
     to/from the EEPROM are the same */
  /* TransferStatus2 = FAILED, if the transmitted and received data 
     to/from the EEPROM are different */


  hal_eeprom_ReadBuffer(Rx3_Buffer, EEPROM_ReadAddress3, BufferSize3);

  TransferStatus3 = Buffercmp(Tx3_Buffer, Rx3_Buffer, BufferSize3);

  while (1)
  {
  }

}
#endif //_TEST_EEPROM_

