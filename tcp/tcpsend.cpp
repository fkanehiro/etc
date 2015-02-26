#include "SocketUtil.h"
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <sys/time.h>

int main(int argc, char *argv[])
{
  int port=2047;
  char *host="tetsujin2", *message;
  int n = 100;
 
  for (int i=1; i<argc; i++){
    if (strcmp(argv[i], "-p")==0){
      port = atoi(argv[++i]);
    }else if(strcmp(argv[i], "-h")==0){
      host = argv[++i];
    }else if(strcmp(argv[i], "-n")==0){
      n = atoi(argv[++i]);
    }
  }

  int socket = open_socket(host, port);

  if (socket < 0){
    std::cerr << "failed to connect(" << host << ":" << port << ")"
	      << std::endl;
    return -1;
  }

  message = "Hello!\n";
  std::cout << "send " << strlen(message) << "bytes for " << n << " times" 
	    << std::endl;
  struct timeval tv1, tv2;
  gettimeofday(&tv1, NULL);
  for (int i=0; i<n; i++){
    send(socket,message,strlen(message),0);
  }
  gettimeofday(&tv2, NULL);
  double time = (tv2.tv_sec - tv1.tv_sec)+(tv2.tv_usec - tv1.tv_usec)/1e6;
  double baudrate = strlen(message)*n*8/time;
  std::cout << "It took " << time << "[s] = " 
	    << baudrate/1e6 << "[Mbps] = "
	    << baudrate/1e3 << "[Kbps]"
	    << std::endl;


  close(socket);
  
  return 0;
}
