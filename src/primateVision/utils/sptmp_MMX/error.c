#include "corr.h"

/*
 * Print the command name followed by an error message specified in the
 * fashion of printf.  If status is non-zero, exit with that status,
 * otherwise return, i.e., status = 0 implies a warning message.
 */

void
Error(int status, const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  (void)fprintf(stderr, "%s: ", cmdname);
  (void)vfprintf(stderr, format, ap);
  (void)fprintf(stderr, "\n");
  if (status)
    _exit(status);
  va_end(ap);
}
