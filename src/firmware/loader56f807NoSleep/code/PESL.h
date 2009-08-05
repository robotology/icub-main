/* ================================================================================================================================= **
** ================================================================================================================================= **
** CONFIGURATION FILE FOR PESL LIBRARY                                                                                               **
** ================================================================================================================================= **
** ================================================================================================================================= */


#define _56852          1
#define _56853          2
#define _56854          3
#define _56855          4
#define _56857          5
#define _56858          6
#define _56F801         7
#define _56F802         8
#define _56F803         9
#define _56F805        10
#define _56F807        11
#define _56F826        12
#define _56F827        13
#define _56F8322       14
#define _56F8323       15
#define _56F8333       16
#define _56F8334       17
#define _56F8345       18
#define _56F8346       19
#define _56F8355       20
#define _56F8356       21
#define _56F8357       22
#define _56F8347       23
#define _56F8365       24
#define _56F8366       25
#define _56F8367       26
#define _56F8122       27
#define _56F8123       28
#define _56F8133       29
#define _56F8134       30
#define _56F8145       31
#define _56F8146       32
#define _56F8155       33
#define _56F8156       34
#define _56F8157       35
#define _56F8147       36
#define _56F8165       37
#define _56F8166       38
#define _56F8167       39
#define _56859         40
#define _56F8011       41
#define _56F8012       42
#define _56F8013       43
#define _56F8014       44

/* Selected target MCU */

#define CPUtype _56F807

/* Include appropriate I/O map file */
#if   (CPUtype == _56852)
  #include "56852.h"
#elif (CPUtype == _56853)
  #include "56853.h"
#elif (CPUtype == _56854)
  #include "56854.h"
#elif (CPUtype == _56855)
  #include "56855.h"
#elif (CPUtype == _56857)
  #include "56857.h"
#elif (CPUtype == _56858)
  #include "56858.h"
#elif (CPUtype == _56859)
  #include "56859.h"
#elif (CPUtype == _56F801)
  #include "56F801.h"
#elif (CPUtype == _56F802)
  #include "56F802.h"
#elif (CPUtype == _56F803)
  #include "56F803.h"
#elif (CPUtype == _56F805)
  #include "56F805.h"
#elif (CPUtype == _56F807)
  #include "56F807.h"
#elif (CPUtype == _56F826)
  #include "56F826.h"
#elif (CPUtype == _56F827)
  #include "56F827.h"
#elif (CPUtype == _56F8322)
  #include "56F8322.h"
#elif (CPUtype == _56F8323)
  #include "56F8323.h"
#elif (CPUtype == _56F8333)
  #include "56F8333.h"
#elif (CPUtype == _56F8334)
  #include "56F8334.h"
#elif (CPUtype == _56F8345)
  #include "56F8345.h"
#elif (CPUtype == _56F8346)
  #include "56F8346.h"
#elif (CPUtype == _56F8355)
  #include "56F8355.h"
#elif (CPUtype == _56F8356)
  #include "56F8356.h"
#elif (CPUtype == _56F8357)
  #include "56F8357.h"
#elif (CPUtype == _56F8347)
  #include "56F8347.h"
#elif (CPUtype == _56F8365)
  #include "56F8365.h"
#elif (CPUtype == _56F8366)
  #include "56F8366.h"
#elif (CPUtype == _56F8367)
  #include "56F8367.h"
#elif (CPUtype == _56F8122)
  #include "56F8122.h"
#elif (CPUtype == _56F8123)
  #include "56F8123.h"
#elif (CPUtype == _56F8133)
  #include "56F8133.h"
#elif (CPUtype == _56F8134)
  #include "56F8134.h"
#elif (CPUtype == _56F8145)
  #include "56F8145.h"
#elif (CPUtype == _56F8146)
  #include "56F8146.h"
#elif (CPUtype == _56F8155)
  #include "56F8155.h"
#elif (CPUtype == _56F8156)
  #include "56F8156.h"
#elif (CPUtype == _56F8157)
  #include "56F8157.h"
#elif (CPUtype == _56F8147)
  #include "56F8147.h"
#elif (CPUtype == _56F8165)
  #include "56F8165.h"
#elif (CPUtype == _56F8166)
  #include "56F8166.h"
#elif (CPUtype == _56F8167)
  #include "56F8167.h"
#elif (CPUtype == _56F8011)
  #include "56F8011.h"
#elif (CPUtype == _56F8012)
  #include "56F8012.h"
#elif (CPUtype == _56F8013)
  #include "56F8013.h"
#elif (CPUtype == _56F8014)
  #include "56F8014.h"
#else
  #error PESL doesn't support selected CPU.
#endif

/* PESL library */

#include "PESLlib.h"
