/**********************************************************************
   udpsend.c : UDP send program
   Jan 6,2001  copyright Takeshi FUJIKI (fujiki@fc-lab.com)
***********************************************************************/
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <strings.h>

#define DEFAULT_PORT 24575
#define DEFAULT_HOSTNAME "tetsujin2"

main(int argc, char *argv[]) {
  int udp_socket;
  struct sockaddr_in dest_addr;
  struct hostent *dest_host;
  char* message, *host=DEFAULT_HOSTNAME;
  int i,port=DEFAULT_PORT,n=100,l=100,interval=0;

  for (i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-h")==0){
      host = argv[++i];
    }else if(strcmp(argv[i], "-n")==0){
      n = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-l")==0){
      l = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-i")==0){
      interval = atoi(argv[++i]);
    }
  }
  printf("destination = %s:%d\n", host, port);
  printf("number(-n) = %d, length(-l) = %d, interval(-i) = %d\n", n, l, interval);

  udp_socket = socket(AF_INET,SOCK_DGRAM,0);

  bzero((char *) &dest_addr, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  dest_host = gethostbyname(host);
  if (!dest_host) {
    printf("invalid hostname(%s)\n", host);
    return -1;
  }
  bcopy(dest_host->h_addr,(char *)&dest_addr.sin_addr,dest_host->h_length);
  dest_addr.sin_port = htons(port);

  char buf[l];
  int ret;
  for (i=0; i<n; i++){
    if ((ret = sendto(udp_socket,buf,l,0,&dest_addr,sizeof(dest_addr)))!=l){
      printf("return value=%d, data length=%d\n", ret, l); 
    };
    if (interval){
      usleep(interval);
    }
  }

  printf("sent %d bytes\n", n*l);
  close(udp_socket);
  return 0;
}
