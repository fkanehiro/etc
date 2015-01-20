#include <arpa/inet.h>

#ifdef __APPLE__
#define MSG_NOSIGNAL 0
#endif

int init_socket(const char *hostname, u_short port, struct sockaddr_in *sa_in);
int open_socket(const char *hostname, u_short port);
int socket_server(u_short port);

