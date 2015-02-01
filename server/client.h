#include <pthread.h>
#ifndef CLIENT_H
#define CLIENT_H
#define IP_LEN 20
typedef struct node
{
  int sock_fd;
  char ip_addr[IP_LEN];
  pthread_t client_pthread_t;
  struct node * next;
  int connected_flag;
}clientlink,*pclientlink;
void ClientLink_Init(pclientlink *p);
void ClientLink_Insert(pclientlink head,pclientlink new_node);
void ClientLink_Remove(pclientlink head,pclientlink old_node);
pthread_t ClientLinek_GetPthread_tByIP(pclientlink head,char *findip);
#endif
