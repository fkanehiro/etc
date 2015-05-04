#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#define DEFAULT_PORT 24575
#define DEFAULT_HOSTNAME "localhost"

main(int argc, char *argv[]) {
  const char *host=DEFAULT_HOSTNAME;
  int port=DEFAULT_PORT,l=100;

  for (int i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-h")==0){
      host = argv[++i];
    }else if(strcmp(argv[i], "-l")==0){
      l = atoi(argv[++i]);
    }
  }
  printf("destination = %s:%d\n", host, port);
  printf("length(-l) = %d\n", l);

  int udp_socket;
  udp_socket = socket(AF_INET,SOCK_DGRAM,0);

  struct sockaddr_in dest_addr;
  bzero((char *) &dest_addr, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;

  struct hostent *dest_host;
  dest_host = gethostbyname(host);
  if (!dest_host) {
    printf("invalid hostname(%s)\n", host);
    return -1;
  }
  bcopy(dest_host->h_addr,(char *)&dest_addr.sin_addr,dest_host->h_length);
  dest_addr.sin_port = htons(port);

  char buf[1024];
  fgets(buf, 1024, stdin);
  printf("buf=[%s]\n", buf);

  int ret;
  if ((ret = sendto(udp_socket,buf,l,0,(sockaddr *)&dest_addr,sizeof(dest_addr)))!=l){
    printf("return value=%d, data length=%d\n", ret, l); 
  };

  close(udp_socket);
  return 0;
}
