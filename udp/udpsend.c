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

#define DEFAULT_PORT 8000
#define DEFAULT_HOSTNAME "tetsujin2"

main(int argc, char *argv[]) {
  int udp_socket;
  struct sockaddr_in dest_addr;
  struct hostent *dest_host;
  char* message, *host=DEFAULT_HOSTNAME;
  int i,port=DEFAULT_PORT,n=100;

  for (i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-h")==0){
      host = argv[++i];
    } 
  }
  printf("destination = %s:%d\n", host, port);

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

  message = "Hello!\n";
  for (i=0; i<n; i++){
    sendto(udp_socket,message,strlen(message),0,&dest_addr,sizeof(dest_addr));
  }
  close(udp_socket);
  return 0;
}
