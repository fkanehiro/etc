#include <cstdio>
#include <netdb.h>
#include <strings.h>
#include <sys/param.h>
#include <unistd.h>
#include <string.h>
#include "SocketUtil.h"

int init_socket(const char *hostname, u_short port, struct sockaddr_in *sa_in)
{
  int sock;
  struct hostent *host_ent;

  if ((sock = socket(PF_INET, SOCK_STREAM, 0))<0){
    perror("socket");
    return sock;
  }
  if ((host_ent = gethostbyname(hostname)) == NULL) {
    fprintf(stderr, "hostname is unknown(%s)\n", hostname);
    return -1;
  }
  memset(sa_in, 0, sizeof(*sa_in));
  sa_in->sin_family = PF_INET;
  sa_in->sin_port = htons(port);
  bcopy(host_ent->h_addr, &(sa_in->sin_addr), host_ent->h_length);
  return sock;
}

int open_socket(const char *hostname, u_short port)
{
  struct sockaddr_in sin;
  int fd;

  if ((fd = init_socket(hostname, port, &sin)) == -1) {
    return -1;
  } 
  if (connect(fd, (struct sockaddr *)&sin, sizeof(sin))==-1) {
    perror("connect");
    return -1;
  }
  fprintf(stderr, "connection established(%s, %d)\n", hostname, port);

  return fd;
}

int socket_server(u_short port)
{
  char hostname[MAXHOSTNAMELEN];
  int s;
  struct sockaddr_in sa_in;

  gethostname(hostname, MAXHOSTNAMELEN);
  if ((s = init_socket(hostname, port, &sa_in)) == -1){
    return -1;
  }
  if (bind(s, (struct sockaddr *)&sa_in, sizeof(sa_in)) == -1){
    perror("bind");
    return -1;
  }
  if (listen(s, 5) == -1) {
    perror("listen");
    return -1;
  }
  fprintf(stderr, "listening %s:%d\n", hostname, port);
  return (s);
}
