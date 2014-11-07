/**********************************************************************
   udpsend.c : UDP send program
   Jan 6,2001  copyright Takeshi FUJIKI (fujiki@fc-lab.com)
***********************************************************************/
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define DEST_PORT 8000
#define DEST_HOSTNAME "tetsujin2"
#define BUFLEN 256

main(int argc, char *argv[]) {
  int udp_socket;
  struct sockaddr_in dest_addr;
  struct hostent *dest_host;
  char* message;

  udp_socket = socket(AF_INET,SOCK_DGRAM,0);

  bzero((char *) &dest_addr, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  dest_host = gethostbyname(DEST_HOSTNAME);
  bcopy(dest_host->h_addr,(char *)&dest_addr.sin_addr,dest_host->h_length);
  dest_addr.sin_port = htons(DEST_PORT);

  message = "Hello!\n";
  sendto(udp_socket,message,strlen(message),0,&dest_addr,sizeof(dest_addr));
  close(udp_socket);
}
