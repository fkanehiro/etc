/**********************************************************************
   udprecv.c : UDP receive program
   Jan 6,2001  copyright Takeshi FUJIKI (fujiki@fc-lab.com)
***********************************************************************/
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <strings.h>

#define DEFAULT_PORT 8000

main(int argc, char *argv[]) {
  int udp_socket;
  struct sockaddr_in from_addr;
  int flag, n, from_len;
  int i, port=DEFAULT_PORT,cnt=0, l=100;

  for (i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    }else if (strcmp(argv[i], "-l")==0){
      l = atoi(argv[++i]);
    }
  }
  printf("port = %d\n", port);

  udp_socket = socket(AF_INET, SOCK_DGRAM,0);

  bzero((char *) &from_addr,sizeof(from_addr));
  from_addr.sin_family = AF_INET;
  from_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  from_addr.sin_port = htons(port);

  bind(udp_socket, (struct sockaddr *) &from_addr, sizeof(from_addr));

  char buf[l];
  flag=0, cnt=0;
  while(1) {
    cnt += recvfrom(udp_socket,buf,l,flag,&from_addr,&from_len);
    printf("received %d bytes\n", cnt);
  }

}
