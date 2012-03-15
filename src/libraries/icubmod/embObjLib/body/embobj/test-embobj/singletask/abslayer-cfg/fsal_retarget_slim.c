

#include <stdio.h>
#include <string.h>
#include <rt_misc.h>
#include <rt_sys.h>

#include "stdint.h"
#include "fsal.h"

//#include "fsal_cfg.h"



#pragma import(__use_no_semihosting_swi)

/* The following macro definitions may be used to translate this file:

  FSAL_USE_STDIO - use standard Input/Output device
          (default is NOT used)
 */

/* Standard IO device handles. */
#define STDIN   0x8001
#define STDOUT  0x8002
#define STDERR  0x8003

/* Standard IO device name defines. */
const char __stdin_name[]  = "STDIN";
const char __stdout_name[] = "STDOUT";
const char __stderr_name[] = "STDERR";

struct __FILE { int handle; /* Add whatever you need here */ };


/*--------------------------- _ttywrch --------------------------------------*/

void _ttywrch (int ch) {
//#ifdef FSAL_USE_STDIO
  fsal_retarget_stdio_putchar(ch);
//#endif
}

/*--------------------------- _sys_open -------------------------------------*/

FILEHANDLE _sys_open (const char *name, int openmode) {
  /* Register standard Input Output devices. */
  if (strcmp(name, "STDIN") == 0) {
    return (STDIN);
  }
  if (strcmp(name, "STDOUT") == 0) {
    return (STDOUT);
  }
  if (strcmp(name, "STDERR") == 0) {
    return (STDERR);
  }
  return (NULL);
}

/*--------------------------- _sys_close ------------------------------------*/

int _sys_close (FILEHANDLE fh) {
  if (fh > 0x8000) {
    return (0);
  }
  return (-1);
}

/*--------------------------- _sys_write ------------------------------------*/

int _sys_write (FILEHANDLE fh, const uint8_t *buf, uint32_t len, int mode) {
//#ifdef FSAL_USE_STDIO
  if (fh == STDOUT) {
    /* Standard Output device. */
    for (  ; len; len--) {
      fsal_retarget_stdio_putchar (*buf++);
    }
    return (0);
  }
  else if(fh == STDERR)
  {
    return(-1);
  }
//#endif
  if (fh > 0x8000) {
    return (-1);
  }
  return (-1);
}

/*--------------------------- _sys_read -------------------------------------*/

int _sys_read (FILEHANDLE fh, uint8_t *buf, uint32_t len, int mode) {
//#ifdef FSAL_USE_STDIO
  if (fh == STDIN) {
    /* Standard Input device. */
    for (  ; len; len--) {
      *buf++ = fsal_retarget_stdio_getchar ();
    }
    return (0);
  }
  else if(fh == STDERR)
  {
    return(-1);
  }
//#endif
  if (fh > 0x8000) {
    return (-1);
  }
  return (-1);
}

/*--------------------------- _sys_istty ------------------------------------*/

int _sys_istty (FILEHANDLE fh) {
  if (fh > 0x8000) {
    return (1);
  }
  return (0);
}

/*--------------------------- _sys_seek -------------------------------------*/

int _sys_seek (FILEHANDLE fh, long pos) {
  if (fh > 0x8000) {
    return (-1);
  }
  return (0);
}

/*--------------------------- _sys_ensure -----------------------------------*/

int _sys_ensure (FILEHANDLE fh) {
  if (fh > 0x8000) {
    return (-1);
  }
  return (-1);
}

/*--------------------------- _sys_flen -------------------------------------*/

long _sys_flen (FILEHANDLE fh) {
  if (fh > 0x8000) {
    return (0);
  }
  return (0);
}

/*--------------------------- _sys_tmpnam -----------------------------------*/

int _sys_tmpnam (char *name, int sig, unsigned maxlen) {
  return (1);
}

/*--------------------------- _sys_command_string ---------------------------*/

char *_sys_command_string (char *cmd, int len) {
  return (cmd);
}

/*--------------------------- _sys_exit -------------------------------------*/

void _sys_exit (int return_code) {
  /* Endless loop. */
  while (1);
}



/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

 



