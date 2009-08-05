/************************************************************************

Little test code for pic-adc (mph) boards.

It listens on the CAN bus. On message arrival:
	len = 1,
   class = 2,
   src = <any>,
   dest = _board_ID
	dati: <channel>

reads the channel <channel> and sends the value back on the CAN bus:
	id = 0x200 + _board_ID << 4 + <any>,
	len = 3,
	dati: <message/channel ID> <most significant byte> <least sig byte>

************************************************************************/

#include "TestMPH.h"
#include "can-18F248.c"
#include "stdlib.h"

// 16MHz clock.
#use delay(clock=16000000)

#define MPH_CLK  PIN_C3		// signal SCLK
#define MPH_SYNC PIN_C2		// signal SYNC
#define MPH_DIN  PIN_C5		// signal DIN

#define ID_BASE 0x200
#define MAX_CHANNELS 32    // max ADC channels

volatile int1 _wait = 0;

#int_AD
AD_isr()
{

}

#int_CANRX1
CANRX1_isr()
{

}

#int_CANRX0
CANRX0_isr()
{

}

// adc_init
// Init the SYNC signal and multiplexes at level 0
void adc_init()
{
   output_low(MPH_SYNC);
}


/*
read_analog(int channel)

Reads the channel identified by <channel>
Parameters : <channel> between 0 and 31.
Returns  : 10 bit value result of the conversion

Selection of the channel from the multiplexer is the following:
      ______ __   _   _     _   ________
SCLK  ______X  \_/ \_/ ..../ \_/________
____     __          ___________________
SYNC  __/  \________/___________________

DIN   -------<DB7><DB6>....<DB0>--------

          __   __
DIN --->  EN | CS | X | A4 | A3 | A2 | A1 | A0

*/

int16 read_analog(int channel)
{
   int i;
   channel &= 0x1F;

   delay_us(2);			   // 2 usec delay
   output_high(MPH_CLK);   // activates SCLK
   output_high(MPH_SYNC);  // activetes SYNC

   delay_us(1);			   // 1 usec delay
   output_low(MPH_SYNC);   // resets SYNC

   for(i=8; i>0; i--)
   {
      if ((channel & (1<<(i-1)))==0)
			output_low(MPH_DIN);
      else
			output_high(MPH_DIN);
      output_low(MPH_CLK);   // resets SCLK
      output_high(MPH_CLK);  // activates SCLK
   }

   delay_us(50);		   // 50 usec delay to stabilize the value.
   return (read_adc());	// reads the analog channel, waits for end of conversion
                        // and returns the value.
}

// This isr is used to enable sending messages on the can bus, it is
// called every 4ms.
//
// reg_val = 65636 - period / ((1/clock) * (4 * divisor))
// reg_val = 65536 - 10 * 10^-3 / (1/16 * 10^-6 * 4 * 8) = 65536 - 5000
//
#INT_TIMER1
void timer1_isr()
{
   set_timer1(65536-5000);
   _wait = 1;
}

/*
void read_all_analog (int16 *buffer)
{
   int i;
   for(i=0; i<32; i++) {
      *(buffer+i)=read_analog(i);
      delay_us(2);
   }
}
*/

void main()
{
   struct rx_stat rxstat;
   int32 rx_id;
   int in_data[8];
   int rx_len;

   int out_data[8];
   int16 tmp;
   int32 tx_id;
   int1 tx_rtr=0;
   int1 tx_ext=0;
   int tx_len=8;
   int tx_pri=3;

   // bitmap, expanded to 8 bit (needed?)
   int8 sequence[MAX_CHANNELS];
   int32 val;

   int i;
   for (i = 0; i < MAX_CHANNELS; i++)
      sequence[i] = MAX_CHANNELS;

   // resource initialization.
   setup_adc_ports(AN0);
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_spi(FALSE);
   setup_wdt(WDT_OFF);
   setup_timer_0(RTCC_INTERNAL);

   setup_timer_1(T1_DISABLED);
   //setup_timer_1(T1_INTERNAL|T1_DIV_BY_1);
   //set_timer1(16000);

   setup_timer_2(T2_DISABLED,0,1);
   setup_timer_3(T3_DISABLED|T3_DIV_BY_1);

   _board_ID = read_eeprom(0);

   can_init();
   adc_init();

   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);

   while(TRUE)
   {
      if (can_kbhit() || _wait)   // wait for a message on the CAN bus
      {
         // handles timer message
         if (_wait)
         {
            _wait = 0;

            for (i = 0; i <= MAX_CHANNELS-3; i+=3)
            {
               if (sequence[i] < MAX_CHANNELS)
                  tmp = read_analog(i);
               else
                  tmp = 0;

               out_data[1] = (int8)(tmp);
               out_data[2] = (int8)(tmp>>8);

               if (sequence[i+1] < MAX_CHANNELS)
                  tmp = read_analog(i+1);
               else
                  tmp = 0;

               out_data[3] = (int8)(tmp);
               out_data[4] = (int8)(tmp>>8);

               if (sequence[i+2] < MAX_CHANNELS)
                  tmp = read_analog(i+2);
               else
                  tmp = 0;

               out_data[5] = (int8)(tmp);
               out_data[6] = (int8)(tmp>>8);

               while (!can_tbe()) ;

               tx_id = ID_BASE;
               tx_id |= ((_board_ID) << 4);
               //tx_id |= ((0 & 0x00f0) >> 4);

               out_data[0] = 0x30+i;
               tx_len = 7;

               can_putd(tx_id, out_data, tx_len, tx_pri, tx_ext, tx_rtr);
            }

            // last message.
            if (sequence[30] < MAX_CHANNELS)
               tmp = read_analog(30);
            else
               tmp = 0;

            out_data[1] = (int8)(tmp);
            out_data[2] = (int8)(tmp>>8);

            if (sequence[31] < MAX_CHANNELS)
               tmp = read_analog(31);
            else
               tmp = 0;

            out_data[3] = (int8)(tmp);
            out_data[4] = (int8)(tmp>>8);

            while (!can_tbe()) ;

            tx_id = ID_BASE;
            tx_id |= ((_board_ID) << 4);
            //tx_id |= ((0 & 0x00f0) >> 4);

            out_data[0] = 0x30+30;
            tx_len = 5;

            can_putd(tx_id, out_data, tx_len, tx_pri, tx_ext, tx_rtr);
         }

         if (can_getd(rx_id, &in_data[0], rx_len, rxstat))
         {
            // handles message for the analog channel
            if ((rx_len == 1) && ((rx_id & 0x700) == 0x200) && (in_data[0] < MAX_CHANNELS))
            {
  		 	      tmp = read_analog(in_data[0]);
               out_data[1] = (int8)(tmp);
               out_data[2] = (int8)(tmp>>8);

               while (!can_tbe()) ;

               tx_id = ID_BASE;
               tx_id |= ((_board_ID) << 4);
               tx_id |= ((rx_id & 0x00f0) >> 4);

               out_data[0] = in_data[0];
			      tx_len = 3;

               // replies to message.
               can_putd(tx_id, out_data, tx_len, tx_pri, tx_ext, tx_rtr);
            }
            // handles message to prepare a sequence (32).
            else
            if ((rx_len == 5) && ((rx_id & 0x700) == 0x200) && (in_data[0] == MAX_CHANNELS))
            {
               for (i = 0; i < MAX_CHANNELS; i++)
                  sequence[i] = MAX_CHANNELS;

               // perhaps this is not needed.
               while (!can_tbe()) ;

               tx_id = ID_BASE;
               tx_id |= ((_board_ID) << 4);
               tx_id |= ((rx_id & 0x00f0) >> 4);

               out_data[0] = in_data[0];
			      tx_len = 1;

               setup_timer_1(T1_DISABLED);

               // replies to message.
               can_putd(tx_id, out_data, tx_len, tx_pri, tx_ext, tx_rtr);
            }
            // handles message to prepare a broadcast sequence (33).
            else
            if ((rx_len == 5) && ((rx_id & 0x700) == 0x200) && (in_data[0] == MAX_CHANNELS+1))
            {
               for (i = 0; i < MAX_CHANNELS; i++)
               {
                  val = *((int32 *)(&in_data[1]));
                  if (val & 0x00000001)
                  {
                     sequence[MAX_CHANNELS-1-i] = MAX_CHANNELS-1-i;
                  }
                  else
                  {
                     sequence[MAX_CHANNELS-1-i] = MAX_CHANNELS;
                  }
                  val >>= 1;
               }

               // perhaps this is not needed.
               // LATER: check the driver.
               while (!can_tbe()) ;

               tx_id = ID_BASE;
               tx_id |= ((_board_ID) << 4);
               tx_id |= ((rx_id & 0x00f0) >> 4);

               out_data[0] = in_data[0];
			      tx_len = 1;

               val = *((int32 *)(&in_data[1]));
               if (val == 0)
               {
                  setup_timer_1(T1_DISABLED);
               }
               else
               {
                  setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);
                  set_timer1(65536-5000);
               }

               // replies to message.
               can_putd(tx_id, out_data, tx_len, tx_pri, tx_ext, tx_rtr);
            }
            // handles CAN bus messages for the downloader
            else
            if ((rx_len == 1) &&  (((rx_id>>8) & 0x7)==7))
            {
               tx_id = (rx_id & 0x0700);
               tx_id |= ((_board_ID) << 4);
               tx_id |= ((rx_id >> 4) & 0x000F);
               switch (in_data[0])
               {
		            case 0xFF:
               		out_data[0] = 0xFF;
               		out_data[1] = 1;  // board type
               		out_data[2] =  read_eeprom(1);  // FW version
               		out_data[3] =  read_eeprom(2);; // FW revision
                      // sends message
               		can_putd(tx_id, out_data, 4,tx_pri,tx_ext,tx_rtr);
               		break;

               	case 0x04:
               		out_data[0] = 4;
               		out_data[1] = 1;
                     // sends message
               		can_putd(tx_id, out_data, 2,tx_pri,tx_ext,tx_rtr);
               		break;

               	case 0:
		            #ASM
         		      GOTO 0x3A00    // jumps to CAN downloader code
            	   #ENDASM
            	}
            }
         }
      }
   }
}
