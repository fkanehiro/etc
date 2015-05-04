#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <strings.h>

#define DEFAULT_PORT 24575

main(int argc, char *argv[]) {
  int udp_socket;
  struct sockaddr_in from_addr;
  int flag, n;
  socklen_t from_len;
  int i, port=DEFAULT_PORT,cnt=0, l=100;

  for (i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    }
  }
  printf("port = %d\n", port);

  udp_socket = socket(AF_INET, SOCK_DGRAM,0);

  bzero((char *) &from_addr,sizeof(from_addr));
  from_addr.sin_family = AF_INET;
  from_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  from_addr.sin_port = htons(port);

  bind(udp_socket, (struct sockaddr *) &from_addr, sizeof(from_addr));

  char buf[1024];
  flag=0;
  while (1){
    recvfrom(udp_socket,buf,1024,flag,(sockaddr *)&from_addr,&from_len);
    printf("%s", buf);
    fflush(stdout);
  }
}
