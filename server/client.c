#include "client.h"
#include <string.h>
#include <stdlib.h>
void ClientLink_Init(pclientlink *p)
{
  (*p) = (clientlink*)malloc(sizeof(clientlink));
  (*p)->next = NULL;
  strcpy((*p)->ip_addr,"head");
}
void ClientLink_Insert(pclientlink head,pclientlink new_node)
{
  pclientlink p = head;
  while(p->next != NULL)
  {
   p = p->next;
  }
  p->next = new_node;
  new_node->next = NULL;
}
void ClientLink_Remove(pclientlink head,pclientlink old_node)
{
  pclientlink p = head,temp;
  while(p->next != NULL)
  {
    temp = p;
    p = p->next;
    if(p == old_node)
    {
       temp->next = p->next;
       free (p);
       break;
    }
  }
}

pthread_t ClientLink_GetPthread_tByIP(pclientlink head, char *findip)
{
   pclientlink p = head;
   while(p->next != NULL)
   {
     p = p->next;
     if(strcmp(p->ip_addr,findip) == 0)
     {
       return p->client_pthread_t;
     }
   }
   return 0;
}
