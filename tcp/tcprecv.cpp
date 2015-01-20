#include "SocketUtil.h"
#include <string.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[])
{
  int port=8000;
 
  for (int i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    } 
  }
  
  int server;
  if ((server = socket_server(port)) == -1){
    std::cerr << "failed to start socket server(port = " << port
              << ")" << std::endl;
    return -1;
  }

  socklen_t fromlen=sizeof(struct sockaddr);
  struct sockaddr fsin;

  while (1){
    int client = accept(server, (struct sockaddr *)&fsin, &fromlen);
    if (client == -1){
      std::cout << "failed to accept" << std::endl;
      return -1;
    }else{
      std::cout << "connected" << std::endl;
    }
    
    int cnt=0;
    while(1){
      int ret;
#define BUFLEN 256
      char buf[BUFLEN];
      
      if ((ret = recv(client, buf, BUFLEN, 0)) <= 0){
	close(client);
	std::cout << "connection is lost" << std::endl;
	break;
      }else{
      }
    }
  }
  return 0;
}
