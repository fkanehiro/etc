#include <stdio.h>
#include <stdlib.h>
#include <err.h>
#define BUF 256
int
main (int argc, char *argv[])
{
  FILE *fp;
  char buf[BUF];
  char *cmdline = "/bin/ls /bin";
  if ( (fp=popen(cmdline,"r")) ==NULL) {
    err(EXIT_FAILURE, "%s", cmdline);
  }
  while(fgets(buf, BUF, fp) != NULL) {
    (void) fputs(buf, stdout);
  }
  (void) pclose(fp);

  exit (EXIT_SUCCESS);
}
