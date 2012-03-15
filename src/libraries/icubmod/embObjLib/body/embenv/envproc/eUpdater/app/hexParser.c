
/* @file       hexParser.c
    @brief      This header file implements the hexParser module.
    @author     marco.accame@iit.it
    @date       05/12/2010
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------



#include "hal.h"

#include "string.h"
#include "stdio.h"

#include "eEcommon.h"
#include "shalPART.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "hexParser.h"


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


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define MAXLINES                        (32*1024)


#define TYPEOFREC_DATA_00               0
#define TYPEOFREC_EOF_01                1
#define TYPEOFREC_SEGADDR_02            2

#define TYPEOFREC_EXTADDR_04            4
#define TYPEOFREC_STARTLINADDR_05       5





// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void s_rewind_file(void);

static uint8_t  s_get08(uint8_t *chr);
static uint16_t s_get16(uint8_t *chr);

// returns 1 only if failure. return 0 in all other cases.
// keep on reading until *eof_flag is 1. if *size is non-zero then copy *data into wherever you want. use *data
// at address *addrdata. 
static uint8_t s_parseline(uint8_t **data, uint8_t *size, uint32_t *addrdata, uint8_t *eof_flag);
static uint8_t s_decode_linehead(uint8_t *linehead, uint8_t *datalen, uint16_t *startaddr, uint8_t *typeofrecord);
static uint8_t s_decode_linedata(uint8_t *linedata, uint8_t datalen, uint8_t *bytedata);
static uint8_t s_decode_baseaddress(uint8_t *linedata, uint32_t *paddress_offset);
static uint8_t s_verify_line(uint8_t datalen, uint16_t addr, uint8_t rectype, uint8_t *data, uint8_t *linetail);



// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static FILE *s_fp = NULL;
static uint32_t s_filelength = 0;
static uint32_t s_filereadbytes = 0;

                                                                             
// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern hexparResult_t hexpar_init(char *filename)
{
    char fdrivename[21] = {0};

    if(NULL == filename)
    {
        return(hexparres_NOK_generic);
    }

    sprintf(fdrivename, "F:\\%s", filename);
    
    if(NULL == s_fp)
    {    
        s_fp = fopen((const char*)fdrivename, "r");
    }
    
    if(NULL == s_fp)
    {
        return(hexparres_NOK_generic);
    }
    
    s_rewind_file();

    return(hexparres_OK);
}


extern hexparResult_t hexpar_deinit(void)
{
    if(NULL != s_fp)
    {
        fclose(s_fp);
        s_fp = NULL;
        s_filelength = 0;
        s_filereadbytes = 0;
    }

    return(hexparres_OK);
}

extern hexparResult_t hexparVerify(const uint32_t addrlow, const uint32_t addrhigh, uint32_t *lower, uint32_t *upper)
{
//    hexparResult_t res = hexparres_NOK_generic;
    uint8_t *data = NULL;
    uint8_t size = 0;
    uint32_t dataaddress = 0;
    uint8_t eof_flag = 0;
    uint8_t foundaddrlow_flag = 0;
    uint8_t beyondaddrhigh_flag = 0;
    uint32_t numberofbytes = 0;
    uint32_t numberoflines = 0;
    uint8_t val;
    
    *lower = UINT32_MAX;
    *upper = 0;

    
    if((NULL == s_fp) || (0 == (addrlow+addrhigh)) || (addrlow == addrhigh))
    {
        return(hexparres_NOK_generic);
    }
    
    
    // now get a line at a time until we reach end of file
    for(;;)
    {

        if(0x58 == numberoflines)
        {
            val = 1;
        }
        val = s_parseline(&data, &size, &dataaddress, &eof_flag);
        
        numberoflines ++;
         
        // any error in file
        if(1 == val)
        {
            return(hexparres_NOK_generic);
        }
       
        // end of file
        if(1 == eof_flag)
        {
            break;
        }
        
        if(numberoflines > MAXLINES)
        {
            return(hexparres_NOK_generic);
        }
    
        // a line with some data 
        if(0 != size)
        {           
            numberofbytes += size;
            
            if(addrlow == dataaddress)
            {
                foundaddrlow_flag = 1;
            }
            
            if((dataaddress+size) > addrhigh)
            {
                beyondaddrhigh_flag = 1;
                return(hexparres_NOK_generic);
            }
            
            if(*lower > dataaddress)
            {
                *lower = dataaddress;
            }
            
            if(*upper < (dataaddress+size))
            {
                *upper = dataaddress+size;
            }
  
        }
   
    }
    
    if(0 == foundaddrlow_flag)
    {
        return(hexparres_NOK_generic);
    }
    if(1 == beyondaddrhigh_flag)
    {
        return(hexparres_NOK_generic);
    }
    
    // everything is ok.
    s_rewind_file();
    return(hexparres_OK);
}


extern hexparResult_t hexparVerifyBin(const uint32_t addrlow, const uint32_t addrhigh, eEmoduleInfo_t **proc)
{
//    hexparResult_t res = hexparres_NOK_generic;

    static eEmoduleInfo_t s_pro;
    uint8_t len = 0;

    
    if((NULL == s_fp) || (0 == (addrlow+addrhigh)) || (addrlow == addrhigh))
    {
        return(hexparres_NOK_generic);
    }

    // simply get the info at location EENV_MODULEINFO_OFFSET

    *proc = &s_pro;

    memset(&s_pro, 0, sizeof(eEmoduleInfo_t)); 

    s_rewind_file();
    fseek(s_fp, EENV_MODULEINFO_OFFSET, SEEK_SET);

    len = fread(&s_pro, 1, sizeof(eEmoduleInfo_t), s_fp);

    if(sizeof(eEmoduleInfo_t) != len)
    {
        return(hexparres_NOK_generic);
    }

    if((s_pro.info.rom.addr != addrlow) || ((s_pro.info.rom.addr + s_pro.info.rom.size) > addrhigh))
    {
        return(hexparres_NOK_generic);
    }
   
    // everything is ok.
    s_rewind_file();
    return(hexparres_OK);
}


extern hexparResult_t hexparRewind(void)
{
    s_rewind_file();
    return(hexparres_OK);
}



// returns -1 only if failure. return 0 in all other cases.
// keep on reading until *eof_flag is 1. if *size is non-zero then copy/use *data into wherever you want. 
// however, *data should be placed at address *addrdata. 
extern hexparResult_t hexparGetData(uint8_t **data, uint8_t *size, uint32_t *addrdata, uint8_t *eof_flag)
{
    uint8_t res;
    uint8_t *d;
    uint8_t s;
    uint32_t a;
    uint8_t e;
    
    res = s_parseline(&d, &s, &a, &e);
    *data = d;
    *size = s;
    *addrdata = a;
    *eof_flag = e;

    return((0 == res) ? (hexparres_OK) : (hexparres_NOK_generic));
}


extern hexparResult_t hexparGetBinData(uint8_t **data, uint16_t *size, uint32_t *addrdata, uint8_t *eof_flag)
{
    uint16_t len;
    uint8_t *d;
    uint8_t s;
    uint32_t a;
    uint8_t e;

    uint32_t pos;

    static uint8_t s_bin_data[2048];


    pos = ftell(s_fp);

    len = fread(s_bin_data, 1, 2048, s_fp);

    if(0 == len)
    {
        *data = NULL;
        *size = 0;
        *addrdata = 0;
        *eof_flag = 1;
    }
    else if(len < 2048)
    {
        *data = s_bin_data;
        *size = len;
        *addrdata = EENV_MEMMAP_EAPPLICATION_ROMADDR + pos;
        *eof_flag = 0;
    }
    else
    {
        *data = s_bin_data;
        *size = 2048;
        *addrdata = EENV_MEMMAP_EAPPLICATION_ROMADDR + pos;
        *eof_flag = 0;
    }
    

    return(hexparres_OK);
}


extern hexparResult_t hexparErasePages(uint32_t lower, uint32_t upper)
{
    uint32_t addr;
    uint32_t pagesize = hal_flash_get_pagesize(lower);

    addr = lower / pagesize;
    addr = addr * pagesize;

    hal_sys_irq_disable();

    // unlock
    hal_flash_unlock();

    // erase
    
    for(; addr<upper; addr +=pagesize)
    {
        hal_flash_erase(addr, pagesize);
    }


    hal_sys_irq_enable();

    return(hexparres_OK);
}

extern hexparResult_t hexparWriteData(uint32_t address, uint8_t *data, uint16_t size)
{

    hal_sys_irq_disable();

    // unlock
    hal_flash_unlock();


    hal_flash_write(address, size, data);


    hal_sys_irq_enable();

    return(hexparres_OK);
}

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------


static void s_rewind_file(void)
{
    if(NULL != s_fp)
    {
        fseek(s_fp, 0, SEEK_END);
        s_filelength = ftell(s_fp);
        fseek(s_fp, 0, SEEK_SET);
        s_filereadbytes = 0;
    }
}

static uint8_t s_get08(uint8_t *chr) 
{
    uint8_t num = 0;
    if(chr[0] <= '9')   num = (chr[0] - '0');
    else                num = (chr[0] - 'A' + 10);

    num <<= 4;

    if(chr[1] <= '9')   num += (chr[1] - '0');
    else                num += (chr[1] - 'A' + 10);

    return(num);
}


static uint16_t s_get16(uint8_t *chr) 
{
    uint16_t num = 0;
    if(chr[0] <= '9')   num = (chr[0] - '0');
    else                num = (chr[0] - 'A' + 10);

    num <<= 4;

    if(chr[1] <= '9')   num += (chr[1] - '0');
    else                num += (chr[1] - 'A' + 10);

    num <<= 4;

    if(chr[2] <= '9')   num += (chr[2] - '0');
    else                num += (chr[2] - 'A' + 10);

    num <<= 4;

    if(chr[3] <= '9')   num += (chr[3] - '0');
    else                num += (chr[3] - 'A' + 10);

    return(num);
}

// returns 1 only if failure. return 0 in all other cases.
// keep on reading until *eof_flag is 1. if *size is non-zero then copy *data into wherever you want. use *data
// at address *addrdata. 
static uint8_t s_parseline(uint8_t **data, uint8_t *size, uint32_t *addrdata, uint8_t *eof_flag)
{
    // a line is :llaaaatt[dd ... repeated ll times]cc  followed by \n\r, for a totale of 9 chars + 2*ll chars + 2 chars + 2
    static uint8_t linehead[9];     // keeps the begin of line which is 9 chars and is :llaaaatt (ll length, aaaa starting addr, tt type of field)
    static uint8_t linedata[510];   // max size is when there is a data fields of 255:
    static uint8_t linetail[4];
    static uint32_t address_offset = 0;
    static uint8_t bytedata[255];

    uint8_t ret;
    uint16_t nread = 0;
    uint8_t datalen = 0;
    uint16_t startaddr = 0;
    uint8_t typeofrecord = 255;
    
 
    // read from file the fixed part:   :llaaaatt
    s_filereadbytes = nread = fread(linehead, 1, 9, s_fp);
    // and decode it.
    ret = s_decode_linehead(linehead, &datalen, &startaddr, &typeofrecord);
    if((9 != nread) || (0 != ret))
    {
        // errore.
        return(1);
    }
    
    switch(typeofrecord)
    {
    
        
        case TYPEOFREC_DATA_00:     
        {
            // retrieve data
            nread = fread(linedata, 1, 2*datalen, s_fp);
            if(nread != 2*datalen)
            {
                return(1);
            }
            s_filereadbytes += nread;
            // convert data in bytes
            ret = s_decode_linedata(linedata, datalen, bytedata);
        } break;
    
        case TYPEOFREC_EOF_01:     
        {
            // retrieve data: no need. there is none
        } break;

        
        case TYPEOFREC_EXTADDR_04:
        {
            // retrieve the offset address
            nread = fread(linedata, 1, 2*datalen, s_fp);
            if(nread != 2*datalen)
            {
                return(1);
            }
            s_filereadbytes += nread;
            // convert data in bytes
            ret = s_decode_linedata(linedata, datalen, bytedata);
            // retrieve the address offset
            ret = s_decode_baseaddress(bytedata, &address_offset);
            
        } break;
        
        case TYPEOFREC_STARTLINADDR_05: 
        {
            // retrieve the offset address
            nread = fread(linedata, 1, 2*datalen, s_fp);
            if(nread != 2*datalen)
            {
                return(1);
            }
            s_filereadbytes += nread;
            // just ignore it. for now.
            ret = s_decode_linedata(linedata, datalen, bytedata);
        } break;       
        
        default:
        case TYPEOFREC_SEGADDR_02: 
        {
            // not supported... think of something better than return error
            return(1);
        } break;        
     
    }
    
    // retrieve the tail bytes
    nread = fread(linetail, 1, 4, s_fp);
    if(4 != nread)
    {
        return(1);
    }
    s_filereadbytes += nread;
    
    ret = s_verify_line(datalen, startaddr, typeofrecord,  bytedata, linetail);
    
    if(0 != ret)
    {
        return(1);
    }
    
    // finish job: give results back.
    *size           = (TYPEOFREC_DATA_00 == typeofrecord) ? (datalen) : (0);
    *data           = bytedata;
    *addrdata       = address_offset + startaddr;
    *eof_flag       = (TYPEOFREC_EOF_01 == typeofrecord) ? (1) : (0);


    return(0);
}

static uint8_t s_decode_linehead(uint8_t *linehead, uint8_t *datalen, uint16_t *startaddr, uint8_t *typeofrecord)
{
    if(':' != linehead[0])
    {
        return(1);
    }
    
    *datalen        = s_get08(&linehead[1]);
    *startaddr      = s_get16(&linehead[3]);
    *typeofrecord   = s_get08(&linehead[7]);
    
    return(0);
}


static uint8_t s_decode_linedata(uint8_t *linedata, uint8_t datalen, uint8_t *bytedata)
{
    uint16_t i;
    
    for(i=0; i<(2*datalen); i+=2)
    {
        bytedata[i/2] = s_get08(&linedata[i]);
    }
    
    return(0);
}


static uint8_t s_decode_baseaddress(uint8_t *bytedata, uint32_t *paddress_offset)
{
    uint16_t tmp;
    
    tmp = *((uint8_t*)&bytedata[0]);
    tmp <<= 8;
    tmp += *((uint8_t*)&bytedata[1]);
   
   
    (*paddress_offset) = (tmp << 16);
    
    return(0);
}


static uint8_t s_verify_line(uint8_t datalen, uint16_t addr, uint8_t rectype, uint8_t *data, uint8_t *linetail)
{
    // to do: sum all bytes of datalen, addr-high, addr-low, rectype, data[i]. then make it modulo 256 and sum 1.
    //        result must be equal to s_get08(linetail)
    uint8_t i;
    uint8_t sum = 0;
    uint8_t chk = 0;
    sum = datalen + (addr&0x00ff) + ((addr>>8)&0x00ff) + rectype;
    for(i=0; i<datalen; i++)
    {
        sum += data[i];
    }
    sum = ~sum;
    sum = sum + 0x01;
 
    chk = s_get08(linetail);
    
    return((sum == chk) ? (0) : (1));
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------


