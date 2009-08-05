#include <ace/OS.h>
#include "driver.h"

//#include "windows.h"
#include <yarp/os/Time.h>

//*****************************************************************/
static char *get_error_str(char *str_buf, long ntstatus)
{
  
  struct ERR2STR
  {
    long ntstatus;
    const char *str;
  }; 
  
  static const struct ERR2STR err2str[]=    
  {
    { NTCAN_SUCCESS            , "NTCAN_SUCCESS"            },
    { NTCAN_RX_TIMEOUT         , "NTCAN_RX_TIMEOUT"         },
    { NTCAN_TX_TIMEOUT         , "NTCAN_TX_TIMEOUT"         },
    { NTCAN_TX_ERROR           , "NTCAN_TX_ERROR"           },
    { NTCAN_CONTR_OFF_BUS      , "NTCAN_CONTR_OFF_BUS"      },
    { NTCAN_CONTR_BUSY         , "NTCAN_CONTR_BUSY"         },
    { NTCAN_CONTR_WARN         , "NTCAN_CONTR_WARN"         }, 
    { NTCAN_NO_ID_ENABLED      , "NTCAN_NO_ID_ENABLED"      },
    { NTCAN_ID_ALREADY_ENABLED , "NTCAN_ID_ALREADY_ENABLED" },
    { NTCAN_ID_NOT_ENABLED     , "NTCAN_ID_NOT_ENABLED"     },
    { NTCAN_INVALID_FIRMWARE   , "NTCAN_INVALID_FIRMWARE"   },
    { NTCAN_MESSAGE_LOST       , "NTCAN_MESSAGE_LOST"       },
    { NTCAN_INVALID_PARAMETER  , "NTCAN_INVALID_PARAMETER"  },
    { NTCAN_INVALID_HANDLE     , "NTCAN_INVALID_HANDLE"     },
#ifdef _WIN32
    { NTCAN_IO_INCOMPLETE      , "NTCAN_IO_INCOMPLETE"      },
    { NTCAN_IO_PENDING         , "NTCAN_IO_PENDING"         },
#endif
    { NTCAN_NET_NOT_FOUND      , "NTCAN_NET_NOT_FOUND"      },
#ifdef _WIN32
    { NTCAN_INVALID_HARDWARE   , "NTCAN_INVALID_HARDWARE"   },
	{ NTCAN_PENDING_WRITE      , "NTCAN_PENDING_WRITE"      },
	{ NTCAN_PENDING_READ       , "NTCAN_PENDING_READ"       },
    { NTCAN_INVALID_DRIVER     , "NTCAN_INVALID_DRIVER"     },
    { NTCAN_OPERATION_ABORTED  , "NTCAN_OPERATION_ABORTED"  },
#endif
    { NTCAN_INSUFFICIENT_RESOURCES, "NTCAN_INSUFFICIENT_RESOURCES"      },
    { 0xffffffff               , "NTCAN_UNKNOWN"            }    /* stop-mark */
  };
  
  const struct ERR2STR *es=err2str;
  
  do
   { 
     if(es->ntstatus == ntstatus) break;
     es ++;
   }
  while((unsigned long)es->ntstatus != 0xffffffff);

  sprintf(str_buf, "%s(0x%08lx)", es->str, ntstatus);

  return str_buf; 
}

//*****************************************************************/
can_parameters_type::can_parameters_type()
{
  /*
  BAUDRATE:
  0 = 1000kb/s
  1 = ???
  2 = 500
  6 = 125
  */

 p_net       =0;
 p_txbufsize =MAX_READ_MSG;
 p_rxbufsize =MAX_READ_MSG;
 p_txtout    =2000;
 p_rxtout    =2000;
 p_mode      =0;
 p_baudrate  =0;
}

///*****************************************************************/
cDriver::cDriver ()
{
 m_net       =0;
 m_txbufsize =MAX_READ_MSG;
 m_rxbufsize =MAX_READ_MSG;
 m_txtout    =2000;
 m_rxtout    =2000;
 m_mode      =0;
 m_baudrate  =0;
}



//*****************************************************************/
int cDriver::init (can_parameters_type* p)
{
 if (p!= NULL)
 {
  m_net       =p->p_net;
  m_txbufsize =p->p_txbufsize;
  m_rxbufsize =p->p_rxbufsize;
  m_txtout    =p->p_txtout;
  m_rxtout    =p->p_rxtout;
  m_mode      =p->p_mode;
  m_baudrate  =p->p_baudrate;
 }
  int i,j=0;

  long  ret;

  char str_buf[100];

  ret = canOpen(m_net, m_mode, m_txbufsize, m_rxbufsize, m_txtout, m_rxtout, &m_h0);
  if(ret != NTCAN_SUCCESS)
    {
      printf("canOpen returned: %s\n", get_error_str(str_buf,ret));
      return(-1);
    }
   else
   {
	  printf("canOpen ok\n");

   }


  ret = canSetBaudrate(m_h0, m_baudrate);
  if(ret != NTCAN_SUCCESS)
     {
       printf("canSetBaudrate returned: %s\n", get_error_str(str_buf,ret));
       return(-1);
     }
     else
   {
	   printf("canSetBaudRate ok\n");

   }
  
  for (i =1700; i<2047; i++)
  {
   ret = canIdAdd(m_h0, i);
   if(ret != NTCAN_SUCCESS)
	  {
	    printf("canIdAdd error on id %d , %s\n", i, get_error_str(str_buf,ret));
	  }
	/*
	 * If driver return NTCAN_INSUFFICIENT_RESOURCES the FIFO is full.
	 * So we sleep some time in order to wait for an empty FIFO
	 */
	if(ret == NTCAN_INSUFFICIENT_RESOURCES)
	  {
	    yarp::os::Time::delay(10);
	    continue;
	  }
  }
  


 return 0;
}

/*
DWORD mtime(void)
{
 DWORD a = GetTickCount();
  printf("current time=%d\n", a);
  return a;
}
*/

#define GET_CURRENT_TIME mtime()
//*****************************************************************/
int cDriver::receive_message(CMSG* messages)
{

  int  ret;
  char str_buf[100];

#ifdef WIN32  
	long how_many_messages=MAX_READ_MSG;
#else 
	int32_t	how_many_messages=MAX_READ_MSG;
#endif 
 
  int i,j;
  
  //DWORD start_time = GET_CURRENT_TIME;
  ret = canRead(m_h0, messages, &how_many_messages, NULL);
  //DWORD stop_time =GET_CURRENT_TIME;
  //printf("Test-Duration=%d \n", (DWORD)(stop_time - start_time));

  if(ret != NTCAN_SUCCESS)
  {
	  printf("read error %s \n", get_error_str(str_buf,ret));
	  return 0;
  }

  return how_many_messages;

  //Here only for debug output
  printf("how_many_messages %d \n", how_many_messages);
  CMSG* cm = NULL;
  for(i=0; i < how_many_messages; i ++)
  {
	  cm =&messages[i];
	  if(cm->len & NTCAN_RTR)
	    { 
         printf("RX-RTR-ID=%9ld (0x%08lx) len=%01X\n",cm->id, cm->id, cm->len & 0x0f );	            
	    }	
	  else
	    {
	      cm->len &= 0x0f;
    
          printf("RX-ID=%9ld (0x%08lx) len=%01X data= ", cm->id, cm->id, cm->len );
          for(j=0; j < cm->len; j++)
             {
			   printf("%02X ", cm->data[j] );
             }
                         
          printf("  [");
             {
               int c;
               for(j=0; j < cm->len; j++)
                   {
                     c= cm->data[j];
                     printf("%c", c > 31 && c < 128 ? c : '.' );
                   }
             }
          printf("]\n");
	   }
  }

  return how_many_messages;

}

//*****************************************************************/
int cDriver::send_message(CMSG& message)
{
#ifdef WIN32  
	long how_many_messages=1;
#else 
	int32_t	how_many_messages=1;
#endif 

  char str_buf[100];

  int ret = canWrite(m_h0, &message, &how_many_messages, NULL);

  if(ret != NTCAN_SUCCESS)
  {
	  printf("write error %s \n", get_error_str(str_buf,ret));
	  return 0;
  }
  else
  return 1;
}
